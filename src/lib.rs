use micro_rdk::common::registry::{ComponentRegistry, RegistryError};

pub mod Stepper28byj48;

pub fn register_models(registry: &mut ComponentRegistry) -> anyhow::Result<(), RegistryError> {
    Stepper28byj48::register_models(registry);
    Ok(())
}