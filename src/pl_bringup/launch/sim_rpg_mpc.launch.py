from pl_bringup.simulation import simulation_launch


def generate_launch_description():
    return simulation_launch('pl_rpg_mpc', 'sim.yaml')
