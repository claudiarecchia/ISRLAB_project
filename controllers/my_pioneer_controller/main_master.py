from controllers.my_pioneer_controller.my_pioneer_controller import Brain
from controllers.my_pioneer_controller.global_variables import TIME_STEP

if __name__ == "__main__":
    brain_master = Brain("CH_placed_spheres_S2M", "CH_placed_spheres_M2S", "CH_current_sphere_S2M", "CH_current_sphere_M2S", "CH_current_coord_S2M", "CH_current_coord_M2S")

    while brain_master.get_robot().step(TIME_STEP) != -1:
        brain_master.sense()
        brain_master.plan()
        brain_master.act()

