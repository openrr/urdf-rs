pub mod ss_vec3 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<'a, D>(deserializer: D) -> Result<[f64; 3], D::Error>
    where
        D: Deserializer<'a>,
    {
        let s = String::deserialize(deserializer)?;
        let vec = s
            .split(' ')
            .filter_map(|x| x.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if vec.len() != 3 {
            return Err(serde::de::Error::custom(format!(
                "failed to parse float array in {s}"
            )));
        }
        let mut arr = [0.0f64; 3];
        arr.copy_from_slice(&vec);
        Ok(arr)
    }
}

pub mod ss_option_vec3 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<'a, D>(deserializer: D) -> Result<Option<[f64; 3]>, D::Error>
    where
        D: Deserializer<'a>,
    {
        let s = String::deserialize(deserializer)?;
        let vec = s
            .split(' ')
            .filter_map(|x| x.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if vec.is_empty() {
            Ok(None)
        } else if vec.len() == 3 {
            let mut arr = [0.0; 3];
            arr.copy_from_slice(&vec);
            Ok(Some(arr))
        } else {
            Err(serde::de::Error::custom(format!(
                "failed to parse float array in {s}"
            )))
        }
    }
}

pub mod ss_vec4 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<'a, D>(deserializer: D) -> Result<[f64; 4], D::Error>
    where
        D: Deserializer<'a>,
    {
        let s = String::deserialize(deserializer)?;
        let vec = s
            .split(' ')
            .filter_map(|x| x.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if vec.len() != 4 {
            return Err(serde::de::Error::custom(format!(
                "failed to parse float array in {s}"
            )));
        }
        let mut arr = [0.0f64; 4];
        arr.copy_from_slice(&vec);
        Ok(arr)
    }
}
