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
"""

import math
import random
import pygame
from pygame.math import Vector2

# ------------------------- Configuration -------------------------
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 700
FPS = 60

ROBOT_RADIUS = 18
FLAME_SENSOR_RANGE = 300   # range to detect and home toward flames
EXTINGUISHER_RANGE = 90    # effective range for extinguishing
OBSTACLE_SENSOR_RANGE = 140
OBSTACLE_SENSOR_FOV = math.radians(90)  # field-of-view for obstacle rays
OBSTACLE_SENSOR_RAYS = 9

MAX_SPEED = 150.0  # pixels/sec
MAX_FORCE = 200.0  # acceleration
DRAG = 0.50
ROTATION_SPEED = 360  # degrees per second when turning

NUM_FIRES = 3
NUM_OBSTACLES = 5
TIME_LIMIT = 120  # seconds

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (150, 150, 150)
DARK_GRAY = (40, 40, 40)
RED = (220, 60, 40)
ORANGE = (255, 140, 0)
BLUE = (60, 140, 220)
GREEN = (60, 200, 100)
YELLOW = (255, 230, 110)

# ------------------------- Helper functions -------------------------

def clamp(n, a, b):
    return max(a, min(b, n))

# ------------------------- Entities -------------------------

class Obstacle:
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)

    def draw(self, surf):
        pygame.draw.rect(surf, DARK_GRAY, self.rect)
        pygame.draw.rect(surf, GRAY, self.rect, 2)

    def collide_point(self, p):
        return self.rect.collidepoint(p)

    def collide_circle(self, center, radius):
        # approximate circle-rect collision
        circle_distance_x = abs(center.x - self.rect.centerx)
        circle_distance_y = abs(center.y - self.rect.centery)
        if circle_distance_x > (self.rect.width/2 + radius):
            return False
        if circle_distance_y > (self.rect.height/2 + radius):
            return False
        if circle_distance_x <= (self.rect.width/2):
            return True
        if circle_distance_y <= (self.rect.height/2):
            return True
        corner_distance_sq = (circle_distance_x - self.rect.width/2)**2 + (circle_distance_y - self.rect.height/2)**2
        return corner_distance_sq <= (radius**2)

class Fire:
    def __init__(self, pos, intensity=100):
        self.pos = Vector2(pos)
        self.intensity = intensity  # 0 -> extinguished
        self.max_intensity = intensity
        self.radius = 18

    def draw(self, surf):
        if self.intensity <= 0:
            return
        t = self.intensity / self.max_intensity
        # inner core
        core_r = int(self.radius * (0.6 + 0.4 * t))
        pygame.draw.circle(surf, ORANGE, (int(self.pos.x), int(self.pos.y)), core_r)
        # outer
        outer_r = int(self.radius * (1.4 + 0.8 * (1 - t)))
        s = pygame.Surface((outer_r*2, outer_r*2), pygame.SRCALPHA)
        alpha = int(200 * t)
        pygame.draw.circle(s, (255, 180, 60, alpha), (outer_r, outer_r), outer_r)
        surf.blit(s, (self.pos.x - outer_r, self.pos.y - outer_r), special_flags=pygame.BLEND_PREMULTIPLIED)
        # intensity bar
        bar_w = 40
        bar_h = 6
        bar_x = self.pos.x - bar_w/2
        bar_y = self.pos.y - outer_r - 10
        pygame.draw.rect(surf, DARK_GRAY, (bar_x, bar_y, bar_w, bar_h))
        pygame.draw.rect(surf, GREEN, (bar_x, bar_y, bar_w * t, bar_h))

class Particle:
    def __init__(self, pos, vel, life=0.6):
        self.pos = Vector2(pos)
        self.vel = Vector2(vel)
        self.life = life
        self.max_life = life

    def update(self, dt):
        self.pos += self.vel * dt
        self.life -= dt

    def draw(self, surf):
        if self.life <= 0:
            return
        t = clamp(self.life / self.max_life, 0, 1)
        r = int(3 * (0.4 + 0.6 * t))
        a = int(255 * t)
        s = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (200, 230, 255, a), (r, r), r)
        surf.blit(s, (self.pos.x - r, self.pos.y - r), special_flags=pygame.BLEND_PREMULTIPLIED)

class Robot:
    def __init__(self, pos):
        self.pos = Vector2(pos)
        self.vel = Vector2(0, 0)
        self.angle = 0  # degrees, 0 = right
        self.angular_vel = 0
        self.sensor_range = FLAME_SENSOR_RANGE
        self.ext_range = EXTINGUISHER_RANGE
        self.particles = []
        self.manual = False

    def forward_vector(self):
        ang = math.radians(self.angle)
        return Vector2(math.cos(ang), math.sin(ang))

    def update_physics(self, dt, obstacles):
        # Apply drag
        self.vel *= DRAG ** dt
        # clamp speed
        if self.vel.length() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        # move and simple collision
        new_pos = self.pos + self.vel * dt
        # check collision with obstacles, simple pushback
        collided = False
        for o in obstacles:
            if o.collide_circle(new_pos, ROBOT_RADIUS):
                collided = True
                # push away along vector from obstacle center
                dir = (new_pos - Vector2(o.rect.center)).normalize()
                new_pos = Vector2(o.rect.center) + dir * (max(o.rect.width, o.rect.height)/2 + ROBOT_RADIUS + 1)
                # damp velocity
                self.vel *= 0.3
        if not collided:
            self.pos = new_pos
        else:
            self.pos = new_pos
        # rotation
        self.angle = (self.angle + self.angular_vel * dt) % 360

    def autonomous_control(self, fires, obstacles, dt):
        # Find nearest active fire within sensor range
        visible_fires = [f for f in fires if f.intensity > 0 and (f.pos - self.pos).length() <= self.sensor_range]
        if not visible_fires:
            # wander behavior (slow drift)
            self.angular_vel = (math.sin(pygame.time.get_ticks()/1000.0) * ROTATION_SPEED * 0.35)
            # small forward thrust
            self.vel += self.forward_vector() * 30 * dt
            return
        # Seek nearest
        target = min(visible_fires, key=lambda f: (f.pos - self.pos).length())
        desired = (target.pos - self.pos)
        distance = desired.length()
        if distance > 1:
            desired_dir = desired.normalize()
        else:
            desired_dir = Vector2(0, 0)
        # obstacle avoidance: cast rays and sum repulsion
        avoidance = Vector2(0, 0)
        for i in range(OBSTACLE_SENSOR_RAYS):
            fraction = (i / (OBSTACLE_SENSOR_RAYS - 1) - 0.5)  # -0.5 .. 0.5
            angle = math.radians(self.angle) + fraction * OBSTACLE_SENSOR_FOV
            ray_dir = Vector2(math.cos(angle), math.sin(angle))
            ray_end = self.pos + ray_dir * OBSTACLE_SENSOR_RANGE
            # check obstacles along ray
            for o in obstacles:
                closest = Vector2(clamp(ray_end.x, o.rect.left, o.rect.right), clamp(ray_end.y, o.rect.top, o.rect.bottom))
                # approximate distance
                to_rect = (closest - self.pos)
                d = to_rect.length()
                if d < OBSTACLE_SENSOR_RANGE and d > 0:
                    # stronger repulsion if nearer
                    repulse = -ray_dir * (OBSTACLE_SENSOR_RANGE - d) / OBSTACLE_SENSOR_RANGE * 200
                    # steer away more from center rays
                    repulse *= (1.0 - abs(fraction)*1.6)
                    avoidance += repulse
        # combine seek and avoidance
        steer = desired_dir * 120.0 + avoidance
        if steer.length() > 0.01:
            steer_angle = math.degrees(math.atan2(steer.y, steer.x))
            # compute shortest angular difference
            diff = (steer_angle - self.angle + 540) % 360 - 180
            self.angular_vel = clamp(diff * 3.0, -ROTATION_SPEED*2, ROTATION_SPEED*2)
            # forward thrust proportional to alignment
            alignment = max(0.0, math.cos(math.radians(diff)))
            self.vel += self.forward_vector() * (120.0 * alignment) * dt
        else:
            self.angular_vel = 0

    def manual_control(self, keys, dt):
        # Arrow keys: rotate and thrust
        turning = 0
        thrust = 0
        if keys[pygame.K_LEFT]:
            turning = -ROTATION_SPEED
        if keys[pygame.K_RIGHT]:
            turning = ROTATION_SPEED
        if keys[pygame.K_UP]:
            thrust = 180
        if keys[pygame.K_DOWN]:
            thrust = -80
        self.angular_vel = turning
        self.vel += self.forward_vector() * thrust * dt

    def extinguish(self, fires, dt):
        # If any fire within extinguisher range and in roughly forward arc, spray particles
        sprayed = False
        for f in fires:
            if f.intensity <= 0:
                continue
            to_fire = f.pos - self.pos
            d = to_fire.length()
            if d <= self.ext_range:
                # angle check (within 100 degrees cone)
                angle_to_fire = math.degrees(math.atan2(to_fire.y, to_fire.x))
                diff = (angle_to_fire - self.angle + 540) % 360 - 180
                if abs(diff) <= 60:
                    # spray particles toward fire
                    sprayed = True
                    num = int(6 + 6 * (1 - d/self.ext_range))
                    for _ in range(num):
                        jitter = Vector2(random.uniform(-20, 20), random.uniform(-20, 20))
                        speed = random.uniform(200, 350)
                        dir = (to_fire.normalize() + jitter*0.01).normalize()
                        vel = dir * speed + Vector2(random.uniform(-40,40), random.uniform(-40,40))
                        p = Particle(self.pos + dir*20, vel, life=0.6)
                        self.particles.append(p)
                    # reduce intensity: stronger when closer
                    f.intensity -= (40.0 * dt) * (1.5 - d/self.ext_range)
                    f.intensity = max(0.0, f.intensity)
        # decay particles and remove
        for p in list(self.particles):
            p.update(dt)
            if p.life <= 0:
                self.particles.remove(p)
        return sprayed

    def draw(self, surf):
        # Draw sensor ranges first (semi-transparent)
        # Flame sensor
        s = pygame.Surface((self.sensor_range*2, self.sensor_range*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 180, 60, 30), (self.sensor_range, self.sensor_range), int(self.sensor_range))
        surf.blit(s, (self.pos.x - self.sensor_range, self.pos.y - self.sensor_range))
        # Extinguisher range
        s2 = pygame.Surface((self.ext_range*2, self.ext_range*2), pygame.SRCALPHA)
        pygame.draw.circle(s2, (100, 200, 255, 40), (self.ext_range, self.ext_range), int(self.ext_range))
        surf.blit(s2, (self.pos.x - self.ext_range, self.pos.y - self.ext_range))

        # Obstacle sensor rays
        for i in range(OBSTACLE_SENSOR_RAYS):
            fraction = (i / (OBSTACLE_SENSOR_RAYS - 1) - 0.5)
            angle = math.radians(self.angle) + fraction * OBSTACLE_SENSOR_FOV
            ray_dir = Vector2(math.cos(angle), math.sin(angle))
            end = self.pos + ray_dir * OBSTACLE_SENSOR_RANGE
            pygame.draw.aaline(surf, (180, 180, 255), (self.pos.x, self.pos.y), (end.x, end.y))

        # Body
        points = []
        fv = self.forward_vector()
        right = fv.rotate(90)
        points.append((self.pos + fv*22).xy)
        points.append((self.pos - fv*14 + right*12).xy)
        points.append((self.pos - fv*14 - right*12).xy)
        pygame.draw.polygon(surf, BLUE, points)
        pygame.draw.polygon(surf, BLACK, points, 2)
        # robot center
        pygame.draw.circle(surf, WHITE, (int(self.pos.x), int(self.pos.y)), 4)

        # draw particles (extinguisher spray)
        for p in self.particles:
            p.draw(surf)

# ------------------------- Simulation Setup -------------------------

def spawn_random_obstacles(n):
    obs = []
    attempts = 0
    while len(obs) < n and attempts < n*30:
        attempts += 1
        w = random.randint(60, 160)
        h = random.randint(40, 120)
        x = random.randint(60, SCREEN_WIDTH - w - 60)
        y = random.randint(60, SCREEN_HEIGHT - h - 60)
        r = pygame.Rect(x, y, w, h)
        # avoid center spawn region
        center_rect = pygame.Rect(SCREEN_WIDTH//2 - 150, SCREEN_HEIGHT//2 - 120, 300, 240)
        overl = False
        if r.colliderect(center_rect):
            overl = True
        for o in obs:
            if r.colliderect(o.rect.inflate(30,30)):
                overl = True
                break
        if not overl:
            obs.append(Obstacle(r))
    return obs


def spawn_fires(n, obstacles, robot_pos):
    fires = []
    attempts = 0
    while len(fires) < n and attempts < n*80:
        attempts += 1
        x = random.randint(60, SCREEN_WIDTH - 60)
        y = random.randint(60, SCREEN_HEIGHT - 60)
        p = Vector2(x, y)
        too_close = False
        if (p - robot_pos).length() < 120:
            too_close = True
        for o in obstacles:
            if o.collide_point((x,y)):
                too_close = True
                break
        for f in fires:
            if (p - f.pos).length() < 80:
                too_close = True
                break
        if not too_close:
            fires.append(Fire(p, intensity=random.randint(90,140)))
    return fires

# ------------------------- Main -------------------------

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Fire-Extinguishing Robot Simulation")
    clock = pygame.time.Clock()

    robot = Robot(Vector2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))

    obstacles = spawn_random_obstacles(NUM_OBSTACLES)
    fires = spawn_fires(NUM_FIRES, obstacles, robot.pos)

    font = pygame.font.SysFont(None, 24)
    bigfont = pygame.font.SysFont(None, 48)

    start_ticks = pygame.time.get_ticks()
    running = True
    success = None

    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    robot.manual = not robot.manual
                if event.key == pygame.K_r:
                    # restart
                    robot = Robot(Vector2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))
                    obstacles = spawn_random_obstacles(NUM_OBSTACLES)
                    fires = spawn_fires(NUM_FIRES, obstacles, robot.pos)
                    start_ticks = pygame.time.get_ticks()
                    success = None

        keys = pygame.key.get_pressed()
        if robot.manual:
            robot.manual_control(keys, dt)
        else:
            robot.autonomous_control(fires, obstacles, dt)

        robot.update_physics(dt, obstacles)
        sprayed = robot.extinguish(fires, dt)

        # check extinguished fires
        remaining = sum(1 for f in fires if f.intensity > 0)
        elapsed = (pygame.time.get_ticks() - start_ticks) / 1000.0
        time_left = max(0, TIME_LIMIT - elapsed)
        if remaining == 0 and success is None:
            success = True
        if time_left <= 0 and success is None:
            success = False

        # draw
        screen.fill((30, 32, 40))

        for o in obstacles:
            o.draw(screen)

        for f in fires:
            f.draw(screen)

        robot.draw(screen)

        # HUD
        hud_y = 8
        txt = font.render(f"Fires remaining: {remaining}", True, WHITE)
        screen.blit(txt, (8, hud_y))
        hud_y += 22
        txt2 = font.render(f"Time left: {int(time_left)}s", True, WHITE)
        screen.blit(txt2, (8, hud_y))
        hud_y += 22
        mode = "MANUAL" if robot.manual else "AUTONOMOUS"
        mode_txt = font.render(f"Mode: {mode} (Press M to toggle, R to restart)", True, WHITE)
        screen.blit(mode_txt, (8, hud_y))

        # show nearest target
        visible = [f for f in fires if f.intensity > 0 and (f.pos - robot.pos).length() <= robot.sensor_range]
        if visible:
            nearest = min(visible, key=lambda f: (f.pos - robot.pos).length())
            pygame.draw.line(screen, YELLOW, (robot.pos.x, robot.pos.y), (nearest.pos.x, nearest.pos.y), 2)

        if success is not None:
            if success:
                msg = bigfont.render("Success! All fires extinguished.", True, GREEN)
            else:
                msg = bigfont.render("Time's up! Failed to extinguish all fires.", True, RED)
            screen.blit(msg, (SCREEN_WIDTH/2 - msg.get_width()/2, SCREEN_HEIGHT/2 - msg.get_height()/2))
            sub = font.render("Press R to restart.", True, WHITE)
            screen.blit(sub, (SCREEN_WIDTH/2 - sub.get_width()/2, SCREEN_HEIGHT/2 + msg.get_height()/2 + 8))

        pygame.display.flip()

    pygame.quit()

if __name__ == '__main__':
    main()
