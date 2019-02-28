# INNOMECH Master Degree - Mobile Robotic Arm for Asbestos Removal

## What is Asbestos and why it is harmful?
Asbestos is a naturally occurring mineral substance that can be pulled into a fluffy consistency. Asbestos fibers are soft and flexible yet resistant to heat, electricity and chemical corrosion. Pure asbestos is an effective insulator, and it can also be mixed into cloth, paper, cement, plastic and other materials to make them stronger. 

These qualities once made asbestos very profitable for business, but unfortunately, they also make asbestos highly toxic.

(https://www.asbestos.com/asbestos/)

## INNOMECH - Mobile Robotic Arm

To remove asbestos many precautions has to be taken before and after. 
<img width="600" alt="Asbestos Removal - https://cdn.nabholz.com/wp-content/uploads/2017/02/Asbestos-Abatement-Header.jpg" src="imgs/Asbestos-Abatement-Header.jpg">


## A Robot To Remove Asbestos Without Human Interaction

Instead of using human force, our robot will do the removing of the asbestos. This will save:
1. **Time:** A human can work maximum 2 hours under those heavy conditions. A machine can work all day, week and year.
2. **Health:** Even with taking best pre-cautions humans can be affected with asbestos. Decreasing the number of humans working with asbestos will also deacrase the health risks.
3. **Money:** A robot can remove asbestos much faster than a human, can reach places humans cannot and does not affected by the asbestos.

|<img width="300" alt="Asbestos Removal - https://cdn.nabholz.com/wp-content/uploads/2017/02/Asbestos-Abatement-Header.jpg" src="imgs/plan_rob.jpg">|<img width="300" alt="Asbestos Removal - https://cdn.nabholz.com/wp-content/uploads/2017/02/Asbestos-Abatement-Header.jpg" src="imgs/IMG_20190207_113329.jpg">|

## Configuration

### Hardware 

In our robot we prefered DYNAMIXEL cards and motors:

|Components:|
|-------|
|
1. OpenCM 9.04C
2. OpenCM EXP 480
3. 6 x MX-64
|<img width="300" alt="Asbestos Removal - https://cdn.nabholz.com/wp-content/uploads/2017/02/Asbestos-Abatement-Header.jpg" src="imgs/cm904c.jpg">|

### Software

The robot is built on Ubuntu 16.04 (Xenial) with ROS Kinetic Kame. For virtual environment Gazebo is used.

## Code Explanation

From this point code will be explained part bu part.

### mra.xacro

### Inertial Macro

To be able to use the gravitational forces in the Gazebo environment we have to define inertial properties of the links. Since we are going to use it for every link in the system we are defining a macro.

The parameters to be given are:
1. mass value
2. inertial matrix values

These values can be obtained from desing files (.stl) or from weighing original equipment.

```xml
<!--Inertial Macro-->
  <xacro:macro name="default_inertial" params="mass ixx:=1 ixy:=0 ixz:=0 iyy:=1 iyz:=0 izz:=1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="${ixx}" ixy="${ixy}" ixz="${ixz}" iyy="${iyy}" iyz="${iyz}" izz="${izz}"/>
    </inertial>
  </xacro:macro>
<!--Inertial Macro-->
```

### Arm Link Macro

Since there are more than one arm link and they are most likely to be the same we are defining a macro for the arm links.

**The parameters:**

1. suffix: Specific name to be given to the arm (link naming follows: armLink${suffix})
2. parent: Parent of the created link (joint naming follows: ${parent}_armLink${suffix}_joint)
3. reflect: Later to be used if arm link wanted to be created in different axis.
4. height: Height of the arm
5. mesh_path: For visualisation, full path of the mesh file
6. joint_type: revolute, prismatic, continuous, fixed (http://wiki.ros.org/urdf/XML/joint)
7. lower_limit, upper_limit: lower and upper limits of the revolute and prismatic joints.
8. axis_x, axis_y, axis_z: axis which revolute joint revolutes around - set to '1'
9. inertia matrix values: Can be obtaiend from design files

For arm links transmission files 'PositionJointInterface' is preferred. While sending Dynamixel radian values will be converted to the torque values.

```xml
<!--Arm Link Macro-->
  <xacro:macro name="arm_link" params="suffix parent reflect height color mesh_path mass joint_type lower_limit upper_limit axis_x axis_y axis_z ixx:=1 ixy:=0 ixz:=0 iyy:=1 iyz:=0 izz:=1">
    <link name="armLink${suffix}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <!--mesh filename="package://innomech/meshes/linked_mx_64t.stl" scale="0.01 0.01 0.01"/-->
          <mesh filename="${mesh_path}" scale="0.01 0.01 0.01"/>
        </geometry>
        <material name="${color}"/>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="${mesh_path}" scale="0.01 0.01 0.01"/>
        </geometry>
      </collision>
      <xacro:default_inertial mass="${mass}" ixx="${ixx}"  ixy="${ixy}" ixz="${ixz}" iyy="${iyy}" iyz="${iyz}" izz="${izz}"/>
    </link> 

    <!-- Joint-->
    <joint name="${parent}_armLink${suffix}_joint" type="${joint_type}">
      <parent link="${parent}"/>
      <child link="armLink${suffix}"/>
      <origin xyz="0 0 ${height}" rpy="0 0 0"/>
      <axis xyz="${axis_x} ${axis_y} ${axis_z}"/>
      <limit velocity="4.8" effort="1" lower="${lower_limit}" upper="${upper_limit}" />
    </joint>

    <transmission name="armLink${suffix}_transmission" type="SimpleTransmission">
      <type>transmission_interface/SimpleTransmission</type>
      <actuator name="$armLink${suffix}_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
      <joint name="${parent}_armLink${suffix}_joint">
        <hardwareInterface>PositionJointInterface</hardwareInterface>
      </joint>
    </transmission>
  </xacro:macro>
  <!--Arm Link Macro-->
```

### Wheel Macro

Altough there are only two wheels, since they are sharing common parameters defined a wheel macro.

**The parameters:**

1. suffix: Specific name to be given to the wheel (link naming follows: wheel_${suffix})
2. parent: Parent of the created wheel (joint naming follows: wheel_${suffix}_joint)
3. reflect: Rotate the wheel correct position (1 for left wheel and -1 for right wheel)
4. height: Radius of the wheel
5. mesh_path: For visualisation, full path of the mesh file
6. joint_type: continuous, unless defined explicitly
7. inertia matrix values: Can be obtaiend from design files

While in arm link transmissions 'PositionJointInterface' is preferred, in wheel's 'VelocityJointInterface' is preferred due to nature of the wheels. 

```xml
<!--Wheel Macro-->
  <xacro:macro name="wheel_link" params="suffix parent reflect height color mesh_path mass joint_type:=continuous ixx:=1 ixy:=0 ixz:=0 iyy:=1 iyz:=0 izz:=1">
    <link name="wheel_${suffix}">
      <visual>
        <origin xyz="0 0 0" rpy="${-reflect*pi/2} 0 0"/>
        <geometry>
          <!--mesh filename="package://innomech/meshes/wheel_left_link2.stl" scale="0.01 0.01 0.01"/-->
          <mesh filename="${mesh_path}" scale="0.01 0.01 0.01"/>
        </geometry>
        <material name="${color}"/>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${-reflect*pi/2} 0 0"/>
        <geometry>
          <cylinder radius="0.06" length="0.025"/>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.05</mu>
              <mu2>0.05</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <xacro:default_inertial mass="${mass}"/>
    </link> 

    <!-- Joint-->
    <joint name="wheel_${suffix}_joint" type="${joint_type}">
      <parent link="${parent}"/>
      <child link="wheel_${suffix}"/>
      <origin xyz="-0.04 ${reflect*0.13} 0.05" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <!--limit velocity="4.8" effort="1" lower="${-pi}" upper="${pi}" /-->
    </joint>

    <transmission name="wheel_${suffix}_transmission" type="SimpleTransmission">
      <type>transmission_interface/SimpleTransmission</type>
      <actuator name="$wheel_${suffix}_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>VelocityJointInterface</hardwareInterface>
      </actuator>

      <joint name="wheel_${suffix}_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
    </transmission>
  </xacro:macro>
<!--Wheel Macro-->
```