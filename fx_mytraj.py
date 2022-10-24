from copy import copy
from pathlib import Path
from sys import path
 
# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
#build_path = Path(__file__).parent.absolute().parent / 'build'
build_path = "/home/fx/ruckig/ruckig/build"
path.insert(0, str(build_path))
 
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
 
 
if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(1, 0.001)  # DoFs, control cycle
    inp = InputParameter(1)
    out = OutputParameter(1)
 
    inp.control_interface = ControlInterface.Velocity
    #inp.control_interface = ControlInterface.Position

    inp.current_position = [0]
    inp.current_velocity = [10]
    inp.current_acceleration = [10]
 
    
    #inp.target_acceleration = [0.0, 0.0, 0.5]
    
    inp.target_position = [-100]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    
    inp.max_velocity = [0.0]
    inp.max_acceleration = [2.0]
    inp.max_jerk = [3.0]
 
 
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))
 
    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
 
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position] + [f'{p:0.3f}' for p in out.new_velocity] ))
        out_list.append(copy(out))
        
        out.pass_to_input(inp)
 
        if not first_output:
            first_output = copy(out)
 
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
 
    # Plot the trajectory
    # path.insert(0, str(Path(__file__).parent.absolute().parent / 'test'))
    # from plotter import Plotter
 
    # Plotter.plot_trajectory(Path(__file__).parent.absolute() / '5_trajectory.pdf', otg, inp, out_list, plot_jerk=False)