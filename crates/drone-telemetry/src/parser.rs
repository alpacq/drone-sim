use crate::error::TelemetryError;
use crate::frame::TelemetryFrame;
use chrono::{DateTime, NaiveDateTime, TimeZone, Utc};
use std::path::Path;

pub fn parse_file(path: &Path) -> Result<Vec<TelemetryFrame>, TelemetryError> {
    let content = std::fs::read_to_string(path).map_err(|e| TelemetryError::Io {
        path: path.display().to_string(),
        source: e,
    })?;
    parse_str(&content)
}

pub fn parse_str(content: &str) -> Result<Vec<TelemetryFrame>, TelemetryError> {
    let blocks = split_blocks(content);
    let mut frames = Vec::with_capacity(blocks.len());

    for (block_idx, block) in blocks.iter().enumerate() {
        match parse_block(block) {
            Some(frame) => frames.push(frame),
            None => {
                // Tolerant parsing — log the skip but continue.
                eprintln!("Warning: block {} skipped (invalid format)", block_idx + 1);
            }
        }
    }

    if frames.is_empty() {
        return Err(TelemetryError::Empty);
    }

    Ok(frames)
}

fn split_blocks(content: &str) -> Vec<String> {
    let mut blocks = Vec::new();
    let mut current = String::new();

    for line in content.lines() {
        if line.trim().is_empty() {
            if !current.trim().is_empty() {
                blocks.push(current.clone());
                current.clear();
            }
        } else {
            current.push_str(line);
            current.push('\n');
        }
    }
    if !current.trim().is_empty() {
        blocks.push(current);
    }
    blocks
}

/// Returns `Some(frame)` when the block is valid, or `None` when it should
/// be skipped.  Errors are intentionally silenced here (tolerant parsing) and
/// logged by the caller.
fn parse_block(block: &str) -> Option<TelemetryFrame> {
    let lines: Vec<&str> = block
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty())
        .collect();

    if lines.len() < 3 {
        return None;
    }

    let index = lines[0].trim().parse::<u32>().ok()?;

    let content = lines[2..].join("\n");
    let content = strip_font_tags(&content);

    let duration_ms = parse_diff_time(&content);
    let timestamp = parse_timestamp(&content);
    let kv = parse_key_values(&content);

    Some(TelemetryFrame {
        index,
        timestamp,
        duration_ms: duration_ms.unwrap_or(33),
        latitude: kv.get("latitude").and_then(|v| v.parse().ok()),
        longitude: kv.get("longitude").and_then(|v| v.parse().ok()),
        rel_alt: kv.get("rel_alt").and_then(|v| v.trim().parse().ok()),
        abs_alt: kv.get("abs_alt").and_then(|v| v.trim().parse().ok()),
        gimbal_yaw: kv.get("gb_yaw").and_then(|v| v.trim().parse().ok()),
        gimbal_pitch: kv.get("gb_pitch").and_then(|v| v.trim().parse().ok()),
        gimbal_roll: kv.get("gb_roll").and_then(|v| v.trim().parse().ok()),
        iso: kv.get("iso").and_then(|v| v.trim().parse().ok()),
        shutter: kv.get("shutter").map(|v| v.trim().to_string()),
        fnum: kv.get("fnum").and_then(|v| v.trim().parse().ok()),
        color_temp: kv.get("ct").and_then(|v| v.trim().parse().ok()),
    })
}

fn strip_font_tags(s: &str) -> String {
    s.replace("<font size=\"28\">", "")
        .replace("</font>", "")
        .replace("<font size='28'>", "")
}

fn parse_diff_time(content: &str) -> Option<u32> {
    let marker = "DiffTime : ";
    let start = content.find(marker)? + marker.len();
    let end = content[start..].find("ms").map(|i| start + i)?;
    content[start..end].trim().parse().ok()
}

fn parse_timestamp(content: &str) -> Option<DateTime<Utc>> {
    for line in content.lines() {
        let line = line.trim();
        if line.len() >= 19 && line.chars().nth(4) == Some('-') && line.chars().nth(7) == Some('-')
        {
            let fmt_ms = "%Y-%m-%d %H:%M:%S%.f";
            let fmt_s = "%Y-%m-%d %H:%M:%S";

            if let Ok(naive) = NaiveDateTime::parse_from_str(line, fmt_ms)
                .or_else(|_| NaiveDateTime::parse_from_str(line, fmt_s))
            {
                return Some(Utc.from_utc_datetime(&naive));
            }
        }
    }
    None
}

fn parse_key_values(content: &str) -> std::collections::HashMap<String, String> {
    let mut map = std::collections::HashMap::new();

    let mut pos = 0;
    while let Some(open) = content[pos..].find('[') {
        let abs_open = pos + open;
        let after_open = abs_open + 1;

        if let Some(close) = content[after_open..].find(']') {
            let abs_close = after_open + close;
            let inner = &content[after_open..abs_close];

            parse_inner_bracket(inner, &mut map);

            pos = abs_close + 1;
        } else {
            break;
        }
    }
    map
}

fn parse_inner_bracket(inner: &str, map: &mut std::collections::HashMap<String, String>) {
    let parts: Vec<&str> = inner.split_whitespace().collect();
    let mut i = 0;

    while i < parts.len() {
        let key_raw = if parts[i].ends_with(':') {
            let k = parts[i].trim_end_matches(':').trim();
            i += 1;
            k
        } else if i + 1 < parts.len() && parts[i + 1] == ":" {
            let k = parts[i].trim();
            i += 2;
            k
        } else {
            i += 1;
            continue;
        };

        if key_raw.is_empty() {
            continue;
        }

        let mut value_parts = Vec::new();
        while i < parts.len() {
            let p = parts[i];
            if p.ends_with(':') || (i + 1 < parts.len() && parts[i + 1] == ":") {
                break;
            }
            value_parts.push(p);
            i += 1;
        }

        if !value_parts.is_empty() {
            map.insert(key_raw.to_lowercase(), value_parts.join(" "));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_BLOCK: &str = r#"1
00:00:00,000 --> 00:00:00,033
<font size="28">SrtCnt : 1, DiffTime : 33ms
2024-03-15 14:22:01.123
[iso : 100] [shutter : 1/1000] [fnum : 280] [ev : 0] [ct : 5500] [color_md : default] [focal_len : 240] [dzoom_ratio: 10000] [latitude: 52.237049] [longitude: 21.017532] [rel_alt: 15.100 abs_alt: 127.600] [gb_yaw: -12.3 gb_pitch: -45.0 gb_roll: 0.0]
</font>"#;

    const SAMPLE_SRT: &str = r#"1
00:00:00,000 --> 00:00:00,033
<font size="28">SrtCnt : 1, DiffTime : 33ms
2024-03-15 14:22:01.123
[iso : 100] [shutter : 1/1000] [fnum : 280] [ev : 0] [ct : 5500] [latitude: 52.237049] [longitude: 21.017532] [rel_alt: 15.100 abs_alt: 127.600] [gb_yaw: -12.3 gb_pitch: -45.0 gb_roll: 0.0]
</font>

2
00:00:00,033 --> 00:00:00,066
<font size="28">SrtCnt : 2, DiffTime : 33ms
2024-03-15 14:22:01.156
[iso : 100] [shutter : 1/1000] [fnum : 280] [ev : 0] [ct : 5500] [latitude: 52.237060] [longitude: 21.017540] [rel_alt: 15.200 abs_alt: 127.700] [gb_yaw: -12.0 gb_pitch: -45.0 gb_roll: 0.0]
</font>"#;

    #[test]
    fn parses_single_block() {
        let frames = parse_str(SAMPLE_BLOCK).unwrap();
        assert_eq!(frames.len(), 1);

        let f = &frames[0];
        assert_eq!(f.index, 1);
        assert_eq!(f.duration_ms, 33);
        assert!((f.latitude.unwrap() - 52.237049).abs() < 1e-6);
        assert!((f.longitude.unwrap() - 21.017532).abs() < 1e-6);
        assert!((f.rel_alt.unwrap() - 15.1).abs() < 0.01);
        assert!((f.gimbal_pitch.unwrap() - (-45.0)).abs() < 0.01);
        assert_eq!(f.iso, Some(100));
    }

    #[test]
    fn parses_multiple_blocks() {
        let frames = parse_str(SAMPLE_SRT).unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].index, 1);
        assert_eq!(frames[1].index, 2);
    }

    #[test]
    fn timestamp_is_parsed() {
        let frames = parse_str(SAMPLE_BLOCK).unwrap();
        assert!(frames[0].timestamp.is_some(), "Timestamp should be parsed");
    }

    #[test]
    fn tolerant_parsing_of_broken_blocks() {
        let broken_srt = "incorrect block\n\n".to_string() + SAMPLE_BLOCK;
        let frames = parse_str(&broken_srt).unwrap();
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn empty_srt_returns_error() {
        let result = parse_str("   \n\n   ");
        assert!(result.is_err());
    }
}
