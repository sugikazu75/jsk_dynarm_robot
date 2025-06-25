- export urdf to `raw_data`
- create `raw_data/actuator_all.csv`
```bash
python scripts/update_actuator_limits_and_dynamics.py raw_data/<ROBOT>.urdf raw_data/actuator_all.csv
```
```bash
python scripts/update_mass_props.py raw_data/<ROBOT>_joint_updated.urdf all_mass_props_from_solidworks.txt
```
```bash
python scripts/parse_actuators_for_gazebo.py raw_data/actuator_all.csv raw_data/actuator_all.urdf

```
```bash
python scripts/remove_collision.py raw_data/<ROBOT>_joint_updated_inerial_updated.urdf
```
- copy `raw_data/<ROBOT>_joint_updated_inerial_updated.urdf` to `<ROBOT>_all.xacro`
- add `xmlns:xacro="http://www.ros.org/wiki/xacro"` to robot tag to change urdf to xacro in <ROBOT>_all.xacro
- copy raw_data/actuator_all.urdf to the tail of `gazebo.xacro`
- create common.xacro
- replace inertial of thrust* link as `xacro:virtual_inertial`
- add mu to `gazebo.xacro`
