

let flock;
let canvas;
//currently only one sheperd
let sheperd;
let sheperds = [];
let up, down, left, right;



function setup() {
  canvas = createCanvas(windowWidth - 20, windowHeight - 100);
  createP("WASD or arrow keys to move.<br>Click to dash.");

  flock = new Flock();
  sheperd = new Sheperd();
  flock.addSheperd(sheperd);
  // Add an initial set of boids into the system
  for (let i = 0; i < 100; i++) {
    let b = new Boid(width / 2,height / 2);
    flock.addBoid(b);
  }
}

function windowResized() {
  resizeCanvas(windowWidth - 20, windowHeight -100);
}



function draw() {
  background(51);
  flock.run();
  sheperd.move();
  sheperd.render();
  // console.log(flock.boids.length);
}

// Add a new boid into the System
// function mouseDragged() {
//   flock.addBoid(new Boid(mouseX, mouseY));
// }

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = [];
  this.sheperds = [];// Initialize the array
}

Flock.prototype.run = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids, this.sheperds);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}

Flock.prototype.addSheperd = function(b) {
  this.sheperds.push(b);
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
  this.sheperddist = 0;
}

Boid.prototype.run = function(boids, sheperds) {
  this.flock(boids, sheperds);
  this.update();
  this.borders();
  this.render();
}

Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids, sheperds) {
  let flee = this.flee(sheperds);
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion
  // Arbitrarily weight these forces
  if (flee.magSq() !== 0.0){
    // console.log("flee = " + flee.magSq());
    // console.log("sep = " + sep.magSq());
  }
  flee.mult(this.sheperddist);
  sep.mult(2.0);
  ali.mult(1.0);
  coh.mult(1.0);
  // Add the force vectors to acceleration
  this.applyForce(flee);
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  let theta = this.velocity.heading() + radians(90);
  fill(127);
  strokeWeight(1);
  stroke(200);
  push();
  translate(this.position.x, this.position.y);
  rotate(theta);
  beginShape();
  vertex(0, -this.r * 2);
  vertex(-this.r, this.r * 2);
  vertex(this.r, this.r * 2);
  endShape(CLOSE);
  pop();
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width + this.r;
  if (this.position.y < -this.r)  this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  let desiredseparation = 25.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0,0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    let steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}

Boid.prototype.flee = function(sheperds) {
  let maxdist = 200.0;
  let steer = createVector(0,0);
  let sum = createVector(0,0);
  let boidPos = createVector(this.position.x, this.position.y);
  let average = 0;
  let counter = 0;
  for (let i = 0; i < sheperds.length; i++){
    let sheperdPos = createVector(sheperds[i].x, sheperds[i].y);
    let v = boidPos.sub(sheperdPos);
    let dist = v.magSq();
    if (dist < sq(maxdist)){
      v.normalize();
      v.mult(maxdist);
      let boidPos = createVector(this.position);
      boidPos.sub(sheperdPos);
      // console.log(boidPos.x + ", " + boidPos.y + ", " + boidPos.magSq());
      v.sub(boidPos);
      steer.add(v);
      average += dist / maxdist;
      counter++;
    }
  }
  if (counter > 0){
    average /= counter;
    average = maxdist - average;
  }
  this.sheperddist = average;
  // console.log(steer.magSq());
  return steer;
}


function Sheperd() {
  this.width = 20;
  this.height = 20;
  this.color = 
  this.x = windowWidth/2 - this.width/2;
  this.y = windowHeight/2 - this.height/2;
  this.direction;
  this.velocity = createVector();
  this.speed = 2;
  this.damping = 0.6;
  this.dashSpeed = 40;
}

Sheperd.prototype.move = function() {
  if (up) {
    this.velocity.y -= this.speed;
  }
  if (down) {
    this.velocity.y += this.speed;
  }
  if (left) {
    this.velocity.x -= this.speed;
  }
  if (right) {
    this.velocity.x += this.speed;
  }
  
  this.x += this.velocity.x;
  this.y += this.velocity.y;
  
  this.velocity.mult(this.damping);

  this.direction = atan2(mouseY - this.y, mouseX - this.x);
}

Sheperd.prototype.dash = function() {
  let a = this.direction;
  this.velocity.add(cos(a) * this.dashSpeed, sin(a) * this.dashSpeed);
}

Sheperd.prototype.render = function() {
  noFill();
  strokeWeight(2);
  stroke(200);
  circle(this.x,this.y, this.width);
  push();
  translate(this.x, this.y);
  rotate(this.direction);
  triangle(20, -10, 20, 10, 40, 0);
  pop();
}

function keyPressed() {
  if (keyCode === 87 || keyCode === 38) {
    up = true;
  }
  if (keyCode === 83 || keyCode === 40) {
    down = true;
  }
  if (keyCode === 65 || keyCode === 37) {
    left = true;
  }
  if (keyCode === 68 || keyCode === 39) {
    right = true;
  }
}

function keyReleased() {
  if (keyCode === 87 || keyCode === 38) {
    up = false;
  }
  if (keyCode === 83 || keyCode === 40) {
    down = false;
  }
  if (keyCode === 65 || keyCode === 37) {
    left = false;
  }
  if (keyCode === 68 || keyCode === 39) {
    right = false;
  }
}

function mouseClicked() {
  sheperd.dash();
}

