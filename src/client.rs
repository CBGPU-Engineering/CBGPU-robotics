use crate::{Result, now, protocol, require, valid_id};
use serde_json::{Value, json};
use std::{
    io::Read,
    time::{Duration, Instant},
};
pub const DEFAULT: &str = "https://cbgpu.com/api/ai/v1";
#[derive(Clone)]
pub struct Client {
    http: reqwest::blocking::Client,
    base: String,
    key: String,
}
impl Client {
    pub fn new(key: String, base: &str, timeout: Duration) -> Result<Self> {
        require(
            !key.is_empty() && !key.contains(['\n', '\r']),
            "Set a private CBGPU API/device key",
        )?;
        let u = reqwest::Url::parse(base)?;
        require(
            u.username().is_empty()
                && u.password().is_none()
                && u.query().is_none()
                && u.fragment().is_none()
                && (u.scheme() == "https"
                    || u.scheme() == "http"
                        && matches!(
                            u.host_str(),
                            Some("localhost" | "127.0.0.1" | "[::1]" | "::1")
                        )),
            "Use HTTPS (HTTP only for loopback tests)",
        )?;
        Ok(Self {
            http: reqwest::blocking::Client::builder()
                .redirect(reqwest::redirect::Policy::none())
                .timeout(timeout)
                .build()?,
            base: base.trim_end_matches('/').into(),
            key,
        })
    }
    pub fn request(
        &self,
        path: &str,
        method: reqwest::Method,
        data: Option<&Value>,
    ) -> Result<Value> {
        require(
            path.starts_with("/robotics/") && !path.contains(['?', '#', '\r', '\n']),
            "Invalid robotics API path",
        )?;
        let mut request = self
            .http
            .request(method, format!("{}{}", self.base, path))
            .bearer_auth(&self.key)
            .header("User-Agent", "CBGPU-Robotics/0.3");
        if let Some(data) = data {
            request = request.json(data);
        }
        let response = request
            .send()
            .map_err(|_| anyhow::anyhow!("Robotics API connection failed"))?;
        let status = response.status();
        require(!status.is_redirection(), "API redirects are not allowed")?;
        let mut bytes = Vec::new();
        response.take(1_048_577).read_to_end(&mut bytes)?;
        require(bytes.len() <= 1_048_576, "API response is too large")?;
        // Do not echo untrusted error bodies, URLs or credentials into operator logs.
        require(
            status.is_success(),
            &format!("CBGPU API {}: request rejected", status.as_u16()),
        )?;
        Ok(serde_json::from_slice(&bytes)?)
    }
    // Positional fields intentionally match the published v1 SDK contract.
    #[allow(clippy::too_many_arguments)]
    pub fn plan(
        &self,
        instruction: &str,
        robot: &str,
        caps: &Value,
        observations: &Value,
        model: &str,
        effort: &str,
        wait: Duration,
    ) -> Result<Value> {
        protocol::capabilities(caps)?;
        protocol::robot_id(robot)?;
        protocol::bounded(observations, 12000, "Observations")?;
        let job=self.request("/robotics/plans",reqwest::Method::POST,Some(&json!({"instruction":instruction,"robot_id":robot,"capabilities":caps,"observations":observations,"observed_at":now(),"model":model,"effort":effort})))?;
        let id = job["id"].as_str().unwrap_or_default();
        require(valid_id(id), "Invalid plan ID")?;
        let path = format!("/robotics/plans/{id}");
        let result = (|| {
            let deadline = Instant::now() + wait;
            while Instant::now() < deadline {
                let r = self.request(&path, reqwest::Method::GET, None)?;
                match r["state"].as_str() {
                    Some("succeeded") => {
                        protocol::envelope(&r["mission"], caps, robot, now())?;
                        return Ok(r["mission"].clone());
                    }
                    Some("failed" | "cancelled" | "deleted") => anyhow::bail!("Planning stopped"),
                    _ => std::thread::sleep(Duration::from_secs(1)),
                }
            }
            anyhow::bail!("Planning timed out waiting for an unreserved GPU")
        })();
        if result.is_err() {
            let _ = self.request(&path, reqwest::Method::DELETE, None);
        }
        result
    }
}
