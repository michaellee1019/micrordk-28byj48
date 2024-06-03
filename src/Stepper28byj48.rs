use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::thread::sleep;

use micro_rdk::{
    common::{
        actuator::{Actuator, ActuatorError},
        board::{Board, BoardType},
        config::ConfigType,
        registry::{get_board_from_dependencies, ComponentRegistry, Dependency, ResourceKey},
        motor::{
            Motor, MotorError, MotorSupportedProperties, MotorType,
            COMPONENT_NAME as MotorCompName,
        },
        status::{Status, StatusError},
    },
    DoCommand,
};

use micro_rdk::google;

const HALF_STEP_FWD: [[bool;4];8] = [
    [false, false, false, true],
    [false, false, true, true],
    [false, false, true, false],
    [false, true, true, false],
    [false, true, false, false],
    [true, true, false, false],
    [true, false, false, false],
    [true, false, false, true],
];

const HALF_STEP_REV: [[bool;4];8] = [
    [true, false, false, true],
    [true, false, false, false],
    [true, true, false, false],
    [false, true, false, false],
    [false, true, true, false],
    [false, false, true, false],
    [false, false, true, true],
    [false, false, false, true],
];

pub(crate) fn register_models(registry: &mut ComponentRegistry) {
    if registry
        .register_motor("28byj48_stepper", &stepper_28byj48_from_config)
        .is_err()
    {
        log::error!("28byj48_stepper is already registered")
    }
    if registry
        .register_dependency_getter(
            MotorCompName,
            "28byj48_stepper",
            &Stepper28byj48::<BoardType>::dependencies_from_config,
        )
        .is_err()
    {
        log::error!("failed to register dependency getter for 28byj48_stepper model")
    }
}

pub(crate) fn stepper_28byj48_from_config(
    cfg: ConfigType,
    deps: Vec<Dependency>,
) -> Result<MotorType, MotorError> {
    
    let board = get_board_from_dependencies(deps)
        .ok_or(MotorError::ConfigError("missing board dependency"))?;
    let motor = Stepper28byj48::<BoardType>::from_config(cfg, board.clone())?.clone();
    Ok(motor)
}

#[derive(DoCommand)]
pub(crate) struct Stepper28byj48<B> {
    board: B,
    in_1: i32,
    in_2: i32,
    in_3: i32,
    in_4: i32,
}

impl<B> Stepper28byj48<B>
where
    B: Board,
{
    pub(crate) fn new(
        in_1: i32,
        in_2: i32,
        in_3: i32,
        in_4: i32,
        board: B,
    ) -> Result<Self, MotorError> {
        let res = Self {
            board,
            in_1,
            in_2,
            in_3,
            in_4,
        };
        Ok(res)
    }

    pub(crate) fn dependencies_from_config(_cfg: ConfigType) -> Vec<ResourceKey> {
        let r_keys = Vec::new();
        r_keys
    }

    pub(crate) fn from_config(cfg: ConfigType, board: BoardType) -> Result<MotorType, MotorError> {
        let in_1 =
            cfg.get_attribute::<i32>("in_1")
                .or(Err(MotorError::ConfigError(
                    "28byj48_stepper, missing 'in_1' attribute",
                )))?;

        let in_2 =
        cfg.get_attribute::<i32>("in_2")
            .or(Err(MotorError::ConfigError(
                "28byj48_stepper, missing 'in_2' attribute",
            )))?;

        let in_3 =
        cfg.get_attribute::<i32>("in_3")
            .or(Err(MotorError::ConfigError(
                "28byj48_stepper, missing 'in_3' attribute",
            )))?;

        let in_4 =
        cfg.get_attribute::<i32>("in_4")
            .or(Err(MotorError::ConfigError(
                "28byj48_stepper, missing 'in_4' attribute",
            )))?;

        Ok(Arc::new(Mutex::new(Stepper28byj48::new(
            in_1, in_2, in_3, in_4, board,
        )?)))
    }

    fn step(&mut self, count:u32, forwards:bool) -> anyhow::Result<(), MotorError>{
        for _ in 0..count {
            let iter = if forwards { HALF_STEP_FWD.iter() } else { HALF_STEP_REV.iter() };
            for step in iter {
                self.board.set_gpio_pin_level(self.in_1,step[0])?;
                self.board.set_gpio_pin_level(self.in_2,step[1])?;
                self.board.set_gpio_pin_level(self.in_3,step[2])?;
                self.board.set_gpio_pin_level(self.in_4,step[3])?;
                sleep(Duration::from_millis(2));
            }
        }
        self.board.set_gpio_pin_level(self.in_1,false)?;
        self.board.set_gpio_pin_level(self.in_2,false)?;
        self.board.set_gpio_pin_level(self.in_3,false)?;
        self.board.set_gpio_pin_level(self.in_4,false)?;
        Ok(())
    }
}

impl<B> Motor for Stepper28byj48<B>
where
    B: Board,
{
    fn set_power(&mut self, pct: f64) -> Result<(), MotorError> {
        let steps = pct.abs() * 512.0;
        if pct < 0.0 {
            self.step(steps as u32, false)?;
        }
        else {
            self.step(steps as u32, true)?;
        }

        Ok(())
    }

    fn get_position(&mut self) -> Result<i32, MotorError> {
        Err(MotorError::MissingEncoder)
    }

    fn go_for(
        &mut self,
        _rpm: f64,
        _revolutions: f64,
    ) -> Result<Option<std::time::Duration>, MotorError> {
        // let (pwr, dur) = go_for_math(self.max_rpm, rpm, revolutions)?;
        // self.set_power(pwr)?;
        // if dur.is_some() {
        //     return Ok(dur);
        // }
        // Ok(None)
        Err(MotorError::MissingEncoder)
    }

    fn get_properties(&mut self) -> MotorSupportedProperties {
        MotorSupportedProperties {
            position_reporting: false,
        }
    }
}

impl<B> Status for Stepper28byj48<B>
where
    B: Board,
{
    fn get_status(&self) -> Result<Option<google::protobuf::Struct>, StatusError> {
        let hm = HashMap::new();
        Ok(Some(micro_rdk::google::protobuf::Struct { fields: hm }))
    }
}

impl<B> Actuator for Stepper28byj48<B>
where
    B: Board,
{
    fn is_moving(&mut self) -> Result<bool, ActuatorError> {
        Ok(false)
    }
    fn stop(&mut self) -> Result<(), ActuatorError> {
        self.set_power(0.0).map_err(|_| ActuatorError::CouldntStop)
    }
}