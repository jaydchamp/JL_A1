#include <iostream>
#include <vector>
#include <string>
using namespace std;

//define this struct to represent positions on the GRID
struct PointOnGrid2D
{
  PointOnGrid2D(int x, int y): x(x), y(y) {}
  void print()
  {
    cout << "(" << x << ", " << y << ")" << endl;
  }

  //x and y values
  int x;
  int y;
};

//define a vector of bools to represent the board
// 0 = dead cell, 1 = alive cell
vector<vector<bool>> gameBoard;

PointOnGrid2D getNorth(PointOnGrid2D point, PointOnGrid2D limits)
{
  //move up one cell (-1 on the y)
  point.y--;

  //if doing this goes over the top edge, wrap around to bottom row
  if(point.y < 0)
  {
    //go to bottom row
    point.y = limits.y - 1;
  }

  //return the point
  return point;
}
PointOnGrid2D getSouth(PointOnGrid2D point, PointOnGrid2D limits)
{
  //move one cell down (+1 on the y)
  point.y++;

  //if doing this goes over the bottom edge, wrap around to top row
  point.y %= limits.y;

  //return the point
  return point;
}
PointOnGrid2D getEast(PointOnGrid2D point, PointOnGrid2D limits)
{
  //move one cell right (+1 on the x)
  point.x++;

  //if doing this goes over the rightside edge, wrap around to the left column
  point.x %= limits.x;

  //return the point
  return point;
}
PointOnGrid2D getWest(PointOnGrid2D point, PointOnGrid2D limits)
{
  //move one cell left (-1 on the x)
  point.x--;

  //if doing this goes over the leftside edge, wrap around to the right column
  if(point.x < 0)
  {
    //go to right column
    point.x = limits.x - 1;
  }

  //return the point
  return point;
}

void printBoard(PointOnGrid2D limits)
{
  //prints the entire board to console
  //first loop thru each row of the board
  for(int lin = 0; lin < limits.y; lin++)
  {
    //then loop thru each cell/column of the selected row
    for(int col = 0; col < limits.x; col++)
    {
      gameBoard[lin][col] ? cout << '#' : cout << '.';
      //check if the cell is alive or dead
      /*if(gameBoard[lin][col])
      {
        //cell is alive, output #
        cout << '#';
      }
      else
      {
        //cell is dead, output .
        cout << '.';
      }*/
    }
    cout << endl;
  }
}

int countNeighbors(PointOnGrid2D p, PointOnGrid2D limits)
{
  int count = 0;

  //calculate coordinates of directly neighboring cells
  auto north = getNorth(p, limits); //north
  auto south = getSouth(p, limits); //south
  auto east = getEast(p, limits); //east
  auto west = getWest(p, limits); //west

  //find the coordinates of the diagonally neighboring cells
  auto northEast = getEast(north, limits); //northeast
  auto northWest = getWest(north, limits); //northwest
  auto southEast = getEast(south, limits); //southeast
  auto southWest = getWest(south, limits); //southwest

  //check if each of the previously found cells is alive
  //if the cell IS ALIVE, add one to the count
  //count = number of alive, neighboring cells
  if(gameBoard[north.y][north.x]) //north
  {
    count++;
  }
  if(gameBoard[south.y][south.x]) //south
  {
    count++;
  }
  if(gameBoard[east.y][east.x]) //east
  {
    count++;
  }
  if(gameBoard[west.y][west.x]) //west
  {
    count++;
  }
  if(gameBoard[northEast.y][northEast.x]) //northeast
  {
    count++;
  }
  if(gameBoard[northWest.y][northWest.x]) //northwest
  {
    count++;
  }
  if(gameBoard[southEast.y][southEast.x]) //southeast
  {
    count++;
  }
  if(gameBoard[southWest.y][southWest.x]) //southwest
  {
    count++;
  }

  //return the number of neighbors found
  return count;
}

void step(PointOnGrid2D limits)
{
  //first create a copy of the current board
  auto newBoard = gameBoard;

  //loop thru each row in the board
  for(int l = 0; l < limits.y; l++)
  {
    //looping thru each element on the current row (looping thru each column)
    for(int c = 0; c < limits.x; c++)
    {
      //find the number of ALIVE neighbors surrounding the current cell
      //use the CountNeighbors function to count the number of surrounding cells
      auto neighbors = countNeighbors({c, l}, limits);
      //also check if the current cell is ALIVE
      auto isAlive = gameBoard[l][c];

      //THEN use the RULES OF LIFE to spread the alive cells and kill the dead ones
      //Rule 1: if the current ALIVE cell, has 2 OR 3 neighbors that are alive...
      if(isAlive && (neighbors == 2 || neighbors == 3))
      {
        //...it survives onto the next generation, and stays alive
        newBoard[l][c] = true;
      }
      //Rule 2: if the current cell is DEAD, and has 2 ALIVE neighbors...
      else if(!isAlive && neighbors == 3)
      {
        //...it becomes ALIVE through reproduction
        newBoard[l][c] = true;
      }
      //Rule 3: (whatever is left):
        //If the cell is already dead and cannot be reproduced
        //If the cell is ALIVE, with more than 3 neighbors, it dies through overpopulation
        //If the cell has less than 2 ALIVE neighbors, it dies through underpopulation
      else
      {
        //kill the cell
        newBoard[l][c] = false;
      }
    }
  }

  //update the GameBoard
  gameBoard = newBoard;
}

int main(){
  //start by defining variables for columns, lines, and number of steps
  int columns, lines, steps;
  //reading first 3 input values from test
  cin >> columns >> lines >> steps;
  //creating an empty gameboard
  gameBoard.resize(lines, vector<bool>(columns, false));

  //next, update the board with the actual input values, given in the console
  //this function read and calculates the 'TESTs' that are pasted into the console
  for(int lin = 0; lin < lines; lin++)
  {
    //cout << "line number: " << lin << endl;
    //read a line, which represents one row of the board
    string line;
    cin >> line;

    //loop through each column of the current row^
    for(int col = 0; col < columns; col++)
    {
      //if the character is # set the cell to true
      if(line[col] == '#')
      {
        //cell is alive!
        gameBoard[lin][col] = true;
      }
      else //else if character is a . set the cell to false
      {
        //cell is dead :(
        gameBoard[lin][col] = false;
      }
    }
  }

  //run the game based on however many steps were input into the console
  for(int i = 0; i < steps; i++)
  {
    //take a step each time using the step function)
    step({columns, lines});
  }

  // print the board
  printBoard({columns, lines});

  return 0;
};