#include <iostream>
#include <iomanip>
#include <vector>
#include <utility>
#include <cmath>
#include <string>

using namespace std;

struct Vector2 {
  double x=0, y=0;
  Vector2() : x(0), y(0){};
  Vector2(double x, double y) : x(x), y(y){};
  Vector2(const Vector2& v) = default;

  // unary operations
  Vector2 operator-() const { return {-x, -y}; }
  Vector2 operator+() const { return {x, y}; }

  // binary operations
  Vector2 operator-(const Vector2& rhs) const { return {x - rhs.x, y - rhs.y}; }
  Vector2 operator+(const Vector2& rhs) const { return {x + rhs.x, y + rhs.y}; }
  Vector2 operator*(const double& rhs) const { return {x * rhs, y * rhs}; }
  friend Vector2 operator*(const double& lhs, const Vector2& rhs) { return {lhs * rhs.x, lhs * rhs.y}; }
  Vector2 operator/(const double& rhs) const { return {x / rhs, y / rhs}; }
  Vector2 operator/(const Vector2& rhs) const { return {x / rhs.x, y / rhs.y}; }
  bool operator!=(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() >= 1.0e-6; };
  bool operator==(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() < 1.0e-6; };

  // assignment operation
  Vector2& operator=(Vector2 const& rhs) = default;
  Vector2& operator=(Vector2&& rhs) = default;

  // compound assignment operations
  Vector2& operator+=(const Vector2& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  Vector2& operator-=(const Vector2& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  Vector2& operator*=(const double& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  Vector2& operator/=(const double& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }
  Vector2& operator*=(const Vector2& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
  }
  Vector2& operator/=(const Vector2& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
  }

  double sqrMagnitude() const { return x * x + y * y; }
  double getMagnitude() const { return sqrt(sqrMagnitude()); }
  static double getMagnitude(const Vector2& vector) { return vector.getMagnitude(); }

  static double Distance(const Vector2& a, const Vector2& b) { return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); };
  double Distance(const Vector2& b) const { return sqrt((x - b.x) * (x - b.x) + (y - b.y) * (y - b.y)); };
  static double DistanceSquared(const Vector2& a, const Vector2& b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); };
  double DistanceSquared(const Vector2& b) const { return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y); };

  static Vector2 normalized(const Vector2& v) { return v.normalized(); };
  Vector2 normalized() const {
    auto magnitude = getMagnitude();

    // If the magnitude is not null
    if (magnitude > 0.)
      return Vector2(x, y) / magnitude;
    else
      return {x, y};
  };

  static const Vector2 zero;
};

const Vector2 Vector2::zero = {0, 0};

struct Boid {
  Boid(const Vector2& pos, const Vector2& vel): position(pos), velocity(vel){};
  Boid():position({0,0}), velocity({0,0}){};
  Vector2 position;
  Vector2 velocity;
};

struct Cohesion {
  double radius; //max radius for cohesion
  double k; //scaling the force

  Cohesion() = default;
  
  //boids = vector of all agents
  //boidAgentIndex = index for the agent that is currently being looked at
  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex)
  {
    Vector2 centerOfMass = {0,0};
    int numNeighbours = 0;

    //go through the list of agents
    for (int i = 0; i < boids.size(); ++i)
    {
      if (i != boidAgentIndex)
      {
        //if the current boid is NOT the starting boid, find the distance to it
        double distance = boids[boidAgentIndex].position.Distance(boids[i].position);
        
        //if the distance is within the radius...
        if(distance <= radius)
        {
          //...increase center of mass and number of neighbors found
          centerOfMass += boids[i].position;
          ++numNeighbours;
        }
      }
    }

    //if no neighbors return zero force
    if(numNeighbours == 0)
    {
      return Vector2(0,0);
    }

    //calculate average center of mass
    centerOfMass /= numNeighbours;

    Vector2 directionToCenter = centerOfMass - boids[boidAgentIndex].position;
    double distanceToCenter = directionToCenter.getMagnitude();

    //compute force of cohesion, capped at the radius 
    //force is scaled to the distance of the center
    double forceMagnitude = k * min(distanceToCenter, radius) / radius;
    Vector2 force = directionToCenter.normalized() * forceMagnitude;

    cout << "Cohesion - Force: " << force.x << " " << force.y << endl;
    return force;
  }
};

struct Alignment {
  double radius; //max radius for alignment
  double k; //scale factor

  Alignment() = default;

  //boids = vector of all agents
  //boidAgentIndex = index for the agent that is currently being looked at
  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex)
  {
    Vector2 avgVelocity = {0,0};
    int numNeighbours = 0;

    // go through the list of agents
    for (int i = 0; i < boids.size(); ++i)
    {
      if(i != boidAgentIndex)
      {
        // if the current boid is NOT the starting boid, find the distance to it
        double distance = boids[boidAgentIndex].position.Distance(boids[i].position);

        // if the distance is within the radius...
        if(distance <= radius)
        {
          //...increase average velocity and number of neighbors 
          avgVelocity += boids[i].velocity;
          ++numNeighbours;
        }
      }
    }

    //include velocity of the agent itself
    avgVelocity += boids[boidAgentIndex].velocity;
    ++numNeighbours;

    //compute force with average velocity
    if(numNeighbours > 0)
    {
      //average velocity
      avgVelocity /= numNeighbours;

      //find direction and normalized velocity
      Vector2 direction = avgVelocity;//.normalized();
      Vector2 currentVelocity = boids[boidAgentIndex].velocity.normalized();

      //compute alignment force and scale by k 
      Vector2 alignForce = (direction - currentVelocity) * k;

      cout << "Alignment - Alignment Force: " << alignForce.x << " " << alignForce.y << endl;
      return alignForce;
    }
    //no force if no neighbors
    return Vector2(0,0);
  }
};

struct Separation {
  double radius; //max radius for alignment
  double k; //for scaling
  double maxForce; //maxmimum allowable magnitude for seperation force

  Separation() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex)
  {
    Vector2 separationForce = {0, 0};
    Vector2 agentPos = boids[boidAgentIndex].position;
    int numNeighbours = 0;

    // boids = vector of all agents
    // boidAgentIndex = index for the agent that is currently being looked at
    for (int i = 0; i < boids.size(); ++i)
    {
      //if its not the current agent
      if(i != boidAgentIndex)
      {
        //and they are within the radius
        double distance = agentPos.Distance(boids[i].position);

        if(distance <= radius)
        {
          //compute separation forces
          //direction to neighbor then the force, add force and num of neighbors
          Vector2 directionToNeighbor = agentPos - boids[i].position;
          Vector2 force = directionToNeighbor / (distance * distance); //changed to distance squared
          separationForce += force;
          ++numNeighbours;
        }
      }
    }

    //if there are any neighbors within the radius compute final force
    if(numNeighbours > 0)
    {
      //average force
      separationForce /= numNeighbours;
      //magnitude of force scaled
      double forceMagnitude = k * separationForce.getMagnitude();

      //make sure it isnt too big
      if(forceMagnitude > maxForce)
      {
        //clamp force
        separationForce = separationForce.normalized() * maxForce;
      }
      else
      {
        //scale by coefficient
        separationForce *= k;
      }
    }
    cout << "Separation - Separation Force: " << separationForce.x << " " << separationForce.y << endl;
    return separationForce;
  }
};

// feel free to edit this main function to meet your needs
int main() {
  // Variable declaration
  Separation separation{};
  Alignment alignment{};
  Cohesion cohesion{};
  int numberOfBoids;
  string line; // for reading until EOF
  vector<Boid> currentState, newState;

  // Input Reading
  cin >> cohesion.radius >> separation.radius >> separation.maxForce >> alignment.radius >> cohesion.k >> separation.k >> alignment.k >> numberOfBoids;
  for (int i = 0; i < numberOfBoids; i++) {
    Boid b;
    cin >> b.position.x >> b.position.y >> b.velocity.x >> b.velocity.y;
    currentState.push_back(b);
    newState.push_back(b);
  }
  cin.ignore(256, '\n');

  while (getline(cin, line)) { // game loop
    //read from the current and store changes in the new state.
    currentState = newState;
    double deltaT = stod(line);
    // a vector of the sum of forces for each boid.
    vector<Vector2> allForces = vector<Vector2>(numberOfBoids, {0, 0});

    // Compute Forces
    for (int i = 0; i < numberOfBoids; i++)  // for every boid
    {
      // Calculate Cohesion Force
      Vector2 cohesionForce = cohesion.ComputeForce(currentState, i);
      // Calculate Separation Force
      Vector2 separationForce = separation.ComputeForce(currentState, i);
      // Calculate Alignment Force
      Vector2 alignmentForce = alignment.ComputeForce(currentState, i);

      // Accumulate the forces
      allForces[i] += cohesionForce + separationForce + alignmentForce;
    }

    // Tick Time and Output
    // todo: edit this. probably my code will be different than yours.
    cout << fixed << setprecision(3);  // set 3 decimal places precision for output

    for (int i = 0; i < numberOfBoids; i++) // for every boid
    {
      newState[i].velocity += allForces[i] * deltaT;
      //newState[i].position += currentState[i].velocity * deltaT;
      newState[i].position += newState[i].velocity * deltaT;
      cout << newState[i].position.x << " " << newState[i].position.y << " "
           << newState[i].velocity.x << " " << newState[i].velocity.y << endl;
    }
    currentState = newState;
  }

  return 0;
}
