
rosrun xacro xacro --inorder -o ../data/worlds/clamp_passing_180deg.urdf ../data/worlds/clamp_passing_180deg.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/clamp_passing_180deg.urdf ../data/worlds/clamp_passing_180deg.dae
rm ../data/worlds/clamp_passing_180deg.urdf

rosrun xacro xacro --inorder -o ../data/worlds/clamp_passing_90deg.urdf ../data/worlds/clamp_passing_90deg.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/clamp_passing_90deg.urdf ../data/worlds/clamp_passing_90deg.dae
rm ../data/worlds/clamp_passing_90deg.urdf


rosrun xacro xacro --inorder -o ../data/worlds/clamp_passing_straight.urdf ../data/worlds/clamp_passing_straight.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/clamp_passing_straight.urdf ../data/worlds/clamp_passing_straight.dae
rm ../data/worlds/clamp_passing_straight.urdf


rosrun xacro xacro --inorder -o ../data/worlds/clamp_passing_multibend.urdf ../data/worlds/clamp_passing_multibend.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/clamp_passing_multibend.urdf ../data/worlds/clamp_passing_multibend.dae
rm ../data/worlds/clamp_passing_multibend.urdf

rosrun xacro xacro --inorder -o ../data/worlds/clamp_passing_multibend2.urdf ../data/worlds/clamp_passing_multibend2.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/clamp_passing_multibend2.urdf ../data/worlds/clamp_passing_multibend2.dae
rm ../data/worlds/clamp_passing_multibend2.urdf


rosrun xacro xacro --inorder -o ../data/worlds/default.urdf ../data/worlds/default.urdf.xacro
rosrun collada_urdf urdf_to_collada ../data/worlds/default.urdf ../data/worlds/default.dae
rm ../data/worlds/default.urdf
