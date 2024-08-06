use pyo3::{pyclass, pyfunction, pymethods};


#[pyclass]
pub struct AudioSamples{
    #[pyo3(get)]
    pub times: Vec<f32>,

    #[pyo3(get)]
    pub channels: Vec<Vec<f32>>,

    #[pyo3(get)]
    pub sample_rate: f64,
}

