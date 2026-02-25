"""
Pygame Fire-Extinguishing Robot Simulation

Features:
- Simple physics for movement (velocity, acceleration, rotation, friction)
- Visible sensors:
  * Flame sensor (large circle showing detection range)
  * Obstacle/UV sensor (front-facing fan of rays showing obstacle distances)
- Robot autonomously seeks nearest flame, navigates around rectangular obstacles
- Extinguisher effect: spray particles that reduce fire intensity while in range
- Fires have an intensity and are extinguished when intensity <= 0
- Obstacles are placed in the world
- Scoring and success/failure: extinguish all fires before timer runs out
- Optional manual control toggle (press M to toggle manual/autonomous)

Run: `python fire_extinguishing_robot.py` (requires pygame: pip install pygame)
