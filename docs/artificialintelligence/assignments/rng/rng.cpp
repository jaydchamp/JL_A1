// add your imports here
#include <fstream>
#include <cstdint>
#include <iostream>
#include <istream>
const std::string TEST_FOLDER = "\\tests\\";
// HELLO PROFESSOR
// I CHOSE TO IMPLEMENT THE Middle-Square Weyl Sequence RNG
// THE CODE IS BELOW
// FROM WIKIPEDIA: "this generator may be the fastest RNG that passes all the statistical tests."

//

/*
unsigned int xorShift(unsigned int seed, int r1, int r2);

int main(){
  // code here
  unsigned int seed, N, min, max;

  std::cin >> seed >> N >> min >> max;

  unsigned int i;
  for(i = N; i >= 1; i--)
  {
    //Run xor shift
    seed = xorShift(seed, min, max);
  }
}
//The purpose of this function is to take the number and xor shift it to output a pseudo-random number
unsigned int xorShift(unsigned int seed, int r1, int r2)
{
  seed = seed xor (seed << 13);
  seed = seed xor (seed >> 17);
  seed = seed xor (seed << 5);

  int value = r1 + (seed % (r2 - r1 + 1)); //clamps the value to between r1 and r2

  //output the new values
  std::cout << value << std::endl;
  return seed;
}
*/


class RNG
{
private:
  uint64_t state; //current state
  uint64_t w;     //Weyl offset
  uint64_t s_const; //Weyl Constant
  uint64_t max_num;
  uint64_t min_num;

public:
  //constructor to initialize RNG with initial state and offset
  RNG(uint64_t s_seed, uint64_t s_offset, uint64_t s_constant, uint64_t max, uint64_t min)
  {
    state = s_seed;
    w = s_offset;
    s_const = s_constant;
    max_num = max;
    min_num = min;
  }

  //Function used to generate the next random number in the set
  uint64_t next()
  {
    //first update Weyl sequence by the constant
    w += s_const;

    //then update the overall state using the middle-squared method
    uint64_t x = state * state;

    //then extract the middle bits from the squared value
    state = (x >> 32);

    //then add the weyl sequence
    state += w;

    //then scale between the max and min values
    state = state % (max_num - min_num + 1);

    //return the state as the next number
    return min_num + state;
  }
};

int main()
{
  unsigned int seed, N, min, max;
  std::cin >> seed >> N >> min >> max;

  unsigned int weyl_offset = 115;
  unsigned int weyl_constant = 321;

  RNG rng(seed,weyl_offset,weyl_constant, max, min);

  unsigned int i;
  for(i = N; i >= 1; i--)
  {
    std::cout << rng.next() << std::endl;
  }
}