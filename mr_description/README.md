## Package name: mr_description
****
### Brief description: this package includes urdf files of joint modules and robots build by using these modules. The robotic system is developed in BIRL lab and is introded in this [paper](http://ieeexplore.ieee.org/document/5354051/).
**Notice**: Mesh files of the module are removed from the repository since owner of this repo are **not authorized** to publish it. Please contact **linkeway@163.com** if you want to use them.
**Naming convention**:
   - There are mainly  three kind of modules: 
      - "T" type. The module axis are perpendicular with axis of the rotatory joint.
      - "I" type. The module axis coincides with axis of the rotatory joint.
      - "G" type. Gripper type.
   - Smaller modules(**diameter is 85mm**) are named begining with a **lower-case** alphbet that indicates the type of the module, e.g. t_module, i_module and g_module.
   - Larger-size modules(**diameter is 100mm**) are named begining with a **upper-case** alphbet that indicates the type of the module, e.g. T_module, G_module and G_module.
   - Since our modules can be set-up in a upside-down way, the upside-down model for each module are created and named with "**_invert**" afer the module type, e.g. T_invert and G_invert.
   - Each joint module is specified in a xacro file.
   - Robots are build using these joint modules, e.g. manipulator5d.urdf.xacro and climbot.urdf.xacro.
