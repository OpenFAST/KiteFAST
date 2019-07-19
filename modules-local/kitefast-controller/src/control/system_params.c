#include "control/system_params.h"
#include "control/system_types.h"

static SystemParams system_params = {
  .power_sensor = {
    .v_380_cal = {
      .scale = 0.13414320054945056,
      .bias = 0.0,
      .bias_count = 0
    },
    .v_batt_48_cal = {
      .scale = 0.01834724135771543,
      .bias = 0.0,
      .bias_count = 0
    },
    .temperature_cal = {
      .scale = 0.25,
      .bias = 0.0,
      .bias_count = 0
    },
    .i_bus_cal = {
      .scale = 0.02446298847695391,
      .bias = -50.100200400801604,
      .bias_count = 0
    },
    .i_release_cal = {
      .scale = -0.30517578125,
      .bias = 156.25,
      .bias_count = 0
    },
    .v_release_cal = {
      .scale = 0.0244140625,
      .bias = 0.0,
      .bias_count = 0
    },
    .v_bus_cal = {
      .scale = 0.33535800137362637,
      .bias = 0.0,
      .bias_count = 0
    }
  },
  .perch = {
    .perched_wing_pos_p = {
      0.4,
      7.8,
      -4.3
    },
    .levelwind_origin_p_0 = {
      0.97,
      0.0,
      1999999999.9835
    },
    .kinetic_friction_perch = 1000.0,
    .I_drum = 8000.0,
    .winch_drum_origin_p = {
      1000000000.0,
      1000000000.0,
      1000000000.0
    },
    .b_drum = 19390.454040948815,
    .b_perch = 0.0,
    .I_perch_and_drum = 8e+21,
    .gsg_pos_wd = {
      1000000000.0,
      1000000000.0,
      1000000000.0
    },
    .heading_cal = {
      .scale = 1.0,
      .bias = -2.44,
      .bias_count = 0
    },
    .perch_origin_g = {
      0.0,
      0.0,
      0.0
    }
  },
  .comms = {
    .udpio = {
      .flight_gear_remote_addr = "127.0.0.1",
      .aio_telemetry_1_remote_port = 27203,
      .aio_telemetry_2_remote_port = 27204,
      .aio_telemetry_remote_addr = "127.255.255.255",
      .joystick_input_remote_port = 30003,
      .aio_telemetry_3_remote_port = 27205,
      .flight_gear_remote_port = 40022
    },
    .aio_port = 40000
  },
  .rotor_sensors = { {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .cal = {
        .scale = -1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }
  },
  .imus = { {
      .gyro_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .dcm_b2m = {
        .d = { {
            1.0,
            0.0,
            0.0
          }, {
            0.0,
            -1.0,
            0.0
          }, {
            0.0,
            0.0,
            -1.0
          }
        }
      },
      .pos = {
        0.39,
        0.085,
        0.534
      },
      .acc_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .mag_cal = { {
          .scale = 0.992986,
          .bias = -0.0386492,
          .bias_count = 0
        }, {
          .scale = 0.999598,
          .bias = -0.00492214,
          .bias_count = 0
        }, {
          .scale = 1.00705,
          .bias = -0.0450955,
          .bias_count = 0
        }
      },
      .pressure_cal = {
        .scale = 100000.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .gyro_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .dcm_b2m = {
        .d = { {
            1.0,
            0.0,
            0.0
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.0,
            0.0,
            1.0
          }
        }
      },
      .pos = {
        0.39,
        -0.085,
        0.438
      },
      .acc_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .mag_cal = { {
          .scale = 0.997705,
          .bias = -0.0164548,
          .bias_count = 0
        }, {
          .scale = 0.992237,
          .bias = 0.0285524,
          .bias_count = 0
        }, {
          .scale = 1.00822,
          .bias = -0.0299486,
          .bias_count = 0
        }
      },
      .pressure_cal = {
        .scale = 100000.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }, {
      .gyro_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .dcm_b2m = {
        .d = { {
            1.0,
            0.0,
            0.0
          }, {
            0.0,
            -1.0,
            0.0
          }, {
            0.0,
            0.0,
            -1.0
          }
        }
      },
      .pos = {
        0.39,
        -0.042,
        0.534
      },
      .acc_cal = { {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }, {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        }
      },
      .mag_cal = { {
          .scale = 0.99918,
          .bias = 0.024215,
          .bias_count = 0
        }, {
          .scale = 0.99817,
          .bias = 0.0020688,
          .bias_count = 0
        }, {
          .scale = 1.00398,
          .bias = -0.0190154,
          .bias_count = 0
        }
      },
      .pressure_cal = {
        .scale = 100000.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }
  },
  .wing_model = 0,
  .rotors = { {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.613,
        3.639,
        1.597
      },
      .version = 5,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = 0.1448,
      .dir = -1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.613,
        1.213,
        1.597
      },
      .version = 6,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = 0.1448,
      .dir = 1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.613,
        -1.213,
        1.597
      },
      .version = 6,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = 0.1448,
      .dir = 1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.613,
        -3.639,
        1.597
      },
      .version = 5,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = 0.1448,
      .dir = -1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.96,
        -3.639,
        -1.216
      },
      .version = 6,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = -0.1501,
      .dir = 1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.96,
        -1.213,
        -1.216
      },
      .version = 5,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = -0.1501,
      .dir = -1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.96,
        1.213,
        -1.216
      },
      .version = 5,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = -0.1501,
      .dir = -1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }, {
      .D = 2.3,
      .I = 1.56,
      .pos = {
        1.96,
        3.639,
        -1.216
      },
      .version = 6,
      .dcm_b2r = {
        .d = { {
            0.9986295347545738,
            0.0,
            -0.05233595624294383
          }, {
            0.0,
            1.0,
            0.0
          }, {
            0.05233595624294383,
            0.0,
            0.9986295347545738
          }
        }
      },
      .local_pressure_coeff = -0.1501,
      .dir = 1,
      .axis = {
        0.9986295347545738,
        0.0,
        0.05233595624294383
      }
    }
  },
  .tether = {
    .linear_density = 0.917,
    .bending_stiffness = 35.0,
    .gsg_ele_to_termination = 0.712,
    .length = 425.8,
    .outer_diameter = 0.0294,
    .tensile_stiffness = 18000000.0,
    .section_drag_coeff = 0.7
  },
  .pitot = {
    .port_angle = 0.47,
    .sensors = { {
        .alpha_cal = {
          .scale = 1.0,
          .bias = 9.5,
          .bias_count = 0
        },
        .beta_cal = {
          .scale = 1.0,
          .bias = 4.3,
          .bias_count = 0
        },
        .max_pressure = 6895.0,
        .stat_cal = {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        },
        .dyn_cal = {
          .scale = 1.0,
          .bias = -2.3,
          .bias_count = 0
        }
      }, {
        .alpha_cal = {
          .scale = 1.0,
          .bias = -3.93,
          .bias_count = 0
        },
        .beta_cal = {
          .scale = 1.0,
          .bias = -3.01,
          .bias_count = 0
        },
        .max_pressure = 600.0,
        .stat_cal = {
          .scale = 1.0,
          .bias = 0.0,
          .bias_count = 0
        },
        .dyn_cal = {
          .scale = 1.0,
          .bias = -5.0,
          .bias_count = 0
        }
      }
    },
    .local_pressure_coeff = 0.168,
    .dcm_b2p = {
      .d = { {
          0.998629534754574,
          0.0,
          0.052335956242944
        }, {
          0.0,
          1.0,
          0.0
        }, {
          -0.052335956242944,
          0.0,
          0.998629534754574
        }
      }
    },
    .pos = {
      3.213,
      0.0,
      0.443
    }
  },
  .wing_serial = 6,
  .ts = 0.01,
  .test_site = 2,
  .gps = { {
      .antenna_dir = {
        0.0,
        -0.342,
        -0.94
      },
      .pos = {
        2.78,
        -0.005,
        0.376
      }
    }, {
      .antenna_dir = {
        0.707,
        0.242,
        -0.665
      },
      .pos = {
        2.969,
        0.005,
        0.391
      }
    }, {
      .antenna_dir = {
        0.0,
        0.0,
        -1.0
      },
      .pos = {
        -0.55,
        -12.68,
        -0.435
      }
    }, {
      .antenna_dir = {
        0.0,
        0.0,
        -1.0
      },
      .pos = {
        -0.55,
        12.68,
        -0.435
      }
    }
  },
  .power_sys = {
    .R_source = 0.1,
    .C_block = 0.006,
    .v_source_0 = 3400.0,
    .use_ground_voltage_compensation = true,
    .P_source = -1100000.0,
    .R_tether = 1.0
  },
  .flight_plan = 8,
  .loadcells = { {
      .channels = { {
          .strain_location = {
            .i_msg = 0,
            .i_strain = 0
          },
          .cal = {
            .scale = 1.0,
            .bias = 0.0,
            .bias_count = 0
          }
        }, {
          .strain_location = {
            .i_msg = 0,
            .i_strain = 1
          },
          .cal = {
            .scale = 1.0,
            .bias = 0.0,
            .bias_count = 0
          }
        }
      },
      .channels_to_force_local_xy = {
        .d = { {
            0.0,
            1.0
          }, {
            -1.0,
            0.0
          }
        }
      },
      .dcm_loadcell2b = {
        .d = { {
            0.057189,
            -0.996195,
            0.065769312096
          }, {
            0.754616,
            -0.0,
            -0.6561661539389999
          }, {
            0.653669,
            0.087156,
            0.75174468612
          }
        }
      }
    }, {
      .channels = { {
          .strain_location = {
            .i_msg = 2,
            .i_strain = 0
          },
          .cal = {
            .scale = 1.0,
            .bias = 0.0,
            .bias_count = 0
          }
        }, {
          .strain_location = {
            .i_msg = 2,
            .i_strain = 1
          },
          .cal = {
            .scale = 1.0,
            .bias = 0.0,
            .bias_count = 0
          }
        }
      },
      .channels_to_force_local_xy = {
        .d = { {
            0.0,
            -1.0
          }, {
            1.0,
            0.0
          }
        }
      },
      .dcm_loadcell2b = {
        .d = { {
            0.050616,
            0.996195,
            0.070952217948
          }, {
            -0.814083,
            -0.0,
            0.580749147201
          }, {
            0.578539,
            -0.087156,
            0.810985414185
          }
        }
      }
    }
  },
  .levelwind = {
    .drum_angle_to_vertical_travel = -0.0052521131220325465,
    .pivot_axis_to_bridle_point = 2.139,
    .elevation_nominal = -0.061086523819801536,
    .elevation_backlash = 0.0,
    .azimuth_offset = -0.5672320068981571,
    .pulley_engage_drum_angle = -3.141592653589793
  },
  .wind_sensor = {
    .dcm_parent2ws = {
      .d = { {
          -0.8013582278294933,
          0.5981847462866082,
          0.0
        }, {
          0.5981847462866082,
          0.8013582278294933,
          0.0
        }, {
          0.0,
          0.0,
          -1.0
        }
      }
    },
    .on_perch = true,
    .pos_parent = {
      2.32,
      0.026,
      -14.9
    }
  },
  .winch = {
    .velocity_cmd_cal = {
      .scale = 1.0,
      .bias = 0.0,
      .bias_count = 0
    },
    .transmission_ratio = 0.0,
    .r_drum = 1.88,
    .position_cal = {
      .scale = 1.88,
      .bias = 0.0,
      .bias_count = 0
    },
    .drum_velocity_cal = {
      .scale = 1.0,
      .bias = 0.0,
      .bias_count = 0
    }
  },
  // .sensor_layout = {
  //   .pitot_fc_labels = {
  //     0,
  //     2
  //   }
  // },
  .remote_perch = {
    .azimuth_offset = 0.0,
    .port_fence_center_local = {
      0.625,
      -4.85,
      0.0
    },
    .fence_dimensions_local = {
      10.0,
      0.2,
      2.9
    },
    .starboard_platform_center_local = {
      -0.625,
      7.35,
      0.0
    },
    .origin_g = {
      -428.8,
      0.0,
      -1.4879999999999995
    },
    .starboard_fence_center_local = {
      -0.625,
      4.85,
      0.0
    },
    .port_platform_center_local = {
      0.625,
      -7.35,
      0.0
    },
    .platform_dimensions_local = {
      10.0,
      5.0,
      2.0
    }
  },
  .gs_model = 1,
  .hitl = {
    .config = {
      .use_software_joystick = false,
      .motor_timeout_sec = 1.0,
      .gs02_timeout_sec = 1.0,
      .motor_level = 1,
      .servo_levels = {
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1
      },
      .gs02_level = 1,
      .sim_level = 0,
      .servo_timeout_sec = 1.0,
      .tether_release_level = 1,
      .tether_release_timeout_sec = 1.0,
      .send_dyno_commands = false
    }
  },
  .gsg = {
    .azi_cal = {
      .scale = 1.0,
      .bias = 0.0,
      .bias_count = 0
    },
    .ele_axis_horiz_offset_g = 0.12,
    .ele_axis_z_g = -0.16,
    .ele_cal = {
      .scale = 1.0,
      .bias = 0.0,
      .bias_count = 0
    }
  },
  .phys = {
    .R_water_vapor = 461.5,
    .g = 9.81,
    .R_dry_air = 287.058,
    .mag_ned = {
      0.274242,
      0.046062,
      0.206902
    },
    .rho = 1.026,
    .P_atm = 90728.3,
    .g_g = {
      0.0,
      0.0,
      9.81
    }
  },
  .joystick = {
    .cal = {
      .yaw = {
        .scale = 1.0,
        .bias = 0.0,
        .bias_count = 0
      },
      .throttle = {
        .scale = 1.0,
        .bias = 0.0,
        .bias_count = 0
      },
      .roll = {
        .scale = 1.0,
        .bias = 0.0,
        .bias_count = 0
      },
      .pitch = {
        .scale = 1.0,
        .bias = 0.0,
        .bias_count = 0
      }
    }
  },
  .servos = { {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 1.0,
      .nonlinear_servo_to_flap_ratio = 0.0
    }, {
      .linear_servo_to_flap_ratio = 0.0,
      .nonlinear_servo_to_flap_ratio = 0.401
    }, {
      .linear_servo_to_flap_ratio = 0.0,
      .nonlinear_servo_to_flap_ratio = 0.401
    }
  },
  .wing = {
    .A = 32.9,
    .bridle_rad = 4.786,
    .c = 1.28,
    .b = 25.66,
    .tail_spike_pos = {
      -8.85,
      0.0,
      0.61
    },
    .bridle_y_offset = -0.5,
    .m_tail = 189.2,
    .proboscis_pos = {
      0.62,
      0.0,
      -0.203
    },
    .remote_perch_peg_pos = { {
        -1.78,
        -7.3,
        -0.07
      }, {
        -1.78,
        7.3,
        -0.07
      }
    },
    .m = 1724.8999999999999,
    .center_of_mass_pos = {
      -0.14,
      -0.006,
      0.104
    },
    .I_inv = {
      .d = { {
          3.065408070623237e-05,
          -7.100940759395175e-08,
          -2.8345193080882066e-08
        }, {
          -7.100940759395175e-08,
          0.00010071577305740756,
          -4.5341621907710654e-08
        }, {
          -2.8345193080882063e-08,
          -4.5341621907710654e-08,
          2.5330194284814775e-05
        }
      }
    },
    .horizontal_tail_pos = {
      -6.776,
      0.045,
      0.8165
    },
    .pylon_pos = { {
        0.8075,
        -3.793,
        0.1535
      }, {
        0.8075,
        -1.367,
        0.1535
      }, {
        0.8075,
        1.06,
        0.1535
      }, {
        0.8075,
        3.486,
        0.1535
      }
    },
    .bridle_pos = { {
        -0.1494,
        -5.8843,
        0.13035
      }, {
        -0.1494,
        5.8661,
        0.13035
      }
    },
    .I = {
      .d = { {
          32622.171249999996,
          23.016634375,
          36.54631875
        }, {
          23.016634375,
          9928.955624999999,
          17.798811875000002
        }, {
          36.54631875,
          17.798811875000002,
          39478.64874999999
        }
      }
    },
    .tail_cg_pos = {
      -5.069,
      0.002,
      0.037
    },
    .i_tail = {
      .d = { {
          152.2,
          -0.5964,
          61.8
        }, {
          -0.5964,
          1356.5,
          0.7337
        }, {
          61.8,
          0.7337,
          1280.2
        }
      }
    },
    .b_pylon = 3.26
  },
  .ground_station = {
    .gps_antenna_pos_g = {
      0.0,
      0.0,
      -6.7
    },
    .gs02 = {
      .racetrack_tether_length = 5.1,
      .perched_wing_pos_p = {
        0.29,
        7.6,
        -4.3
      },
      .drum_origin_p = {
        0.417,
        0.0,
        -3.05
      },
      .anchor_arm_length = 0.484,
      .detwist_elevation = 0.39269908169872414,
      .drum_radius = 1.88,
      .gsg_pos_drum = {
        -2.1,
        0.0,
        1.348
      },
      .boom_azimuth_p = 1.5707963267948966
    },
    .azi_ref_offset = 0.0,
    .heading = 0.0,
    .ground_z = 6.122
  },
  .test_site_params = { // Added this due to outdated system_params.c file - Jmiller STI
    .azi_allow_start = 0.2617993877991494 ,
    .azi_allow_end = -1.5882496193148399,
    .azi_no_go_size = 1.8500490071139892
  }
};

const GlobalSystemParams g_sys = {
  .ts = &system_params.ts,
  .phys = &system_params.phys,
  .tether = &system_params.tether,
  .wing = &system_params.wing,
  .perch = &system_params.perch,
  .rotors = {
    &system_params.rotors[0],
    &system_params.rotors[1],
    &system_params.rotors[2],
    &system_params.rotors[3],
    &system_params.rotors[4],
    &system_params.rotors[5],
    &system_params.rotors[6],
    &system_params.rotors[7]
  }
};

const SystemParams *GetSystemParams(void) { return &system_params; }
SystemParams *GetSystemParamsUnsafe(void) { return &system_params; }
