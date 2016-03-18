### Topics

- objects
  - rigid bodies
  - collision shapes
  - visual representation
  - joints/constraints
- simulation environment (space)
- teleoperation
  - manual
  - guarded
  - shared autonomy
- collision detection
  - broad phase
    - AABB
    - Circles
  - sampling
    - Van der Corput
- visualization
- planning
  - path planning
    - A*
    - PRM
  - trajectories
  - motion planning
- kinematics
  - forward
  - inverse
- controllers
  - PID
- perception
  - sensor types
    - image
    - tactile
    - odometry
    - location
  - computer vision
- manipulation
  - collision manifold
  - grasping




> - **Rigid Bodies**: A rigid body holds the physical properties of an object. (mass, position, rotation, velocity, etc.)
>   It does not have a shape until you attach one or more collision shapes to it. If you’ve done physics with
>   particles before, rigid bodies differ in that they are able to rotate. Rigid bodies generally tend to have
>   a 1:1 correlation to sprites in a game. You should structure your game so that you use the position and
>   rotation of the rigid body for drawing your sprite.
> - **Collision Shapes**: By attaching shapes to bodies, you can define the a body’s shape. You can attach as many
>   shapes to a single body as you need to in order to define a complex shape. Shapes contain the surface
>   properties of an object such as how much friction or elasticity it has.
> - **Constraints/Joints**: Constraints and joints describe how bodies are attached to each other.
> - **Spaces**: Spaces are containers for simulating objects in Chipmunk. You add bodies, shapes
>   and joints to a space and then update the space as a whole. They control how all the rigid
>   bodies, shapes, and constraints interact together.
>
> -- <cite>[Chipmunk Documentation](http://chipmunk-physics.net/release/ChipmunkLatest-Docs/#Basics-Overview)</cite>
