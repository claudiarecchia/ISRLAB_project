from controllers.my_pioneer_controller.my_pioneer_controller import Brain

TIME_STEP = 64
if __name__ == "__main__":

    brain_slave = Brain("CH_placed_spheres_M2S", "CH_placed_spheres_S2M", "CH_current_sphere_M2S",
                        "CH_current_sphere_S2M", "CH_current_coord_M2S", "CH_current_coord_S2M")
    while brain_slave.get_robot().step(TIME_STEP) != -1:
        brain_slave.read_placed_spheres()
        brain_slave.read_current_sphere_other_robot()
        brain_slave.read_current_gps_other_robot()
        brain_slave.controller()
        brain_slave.write_current_sphere()
        brain_slave.write_current_gps()
