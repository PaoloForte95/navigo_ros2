#include <navigo2_generic/random.hpp>
#include <iostream>
#include <ctime>


using namespace std;

int main()
{
  {
  navigo2_generic::UniformIntWithoutRep random(0, 20, 0);

  for (int i = 0; i < 10; i++)
    {
      std::cout << "[" << i << "] : " << random.drawSample() << std::endl;
    }
  }

  {
  navigo2_generic::UniformIntWithoutRep random(0, 20, 0);

  std::cout << "----------------------------------------------" << std::endl;
  for (int i = 0; i < 10; i++)
    {
      std::cout << "[" << i << "] : " << random.drawSample() << std::endl;
    }
  }

  std::cout << "----------------------------------------------" << std::endl;
  std::cout << "----------------------------------------------" << std::endl;
  // This should really vary
  {
  navigo2_generic::UniformIntWithoutRep random(0, 20, 123450987);

  for (int i = 0; i < 10; i++)
    {
      std::cout << "[" << i << "] : " << random.drawSample() << std::endl;
    }
  }

  std::cout << "----------------------------------------------" << std::endl;
  {
    navigo2_generic::UniformIntWithoutRep random(0, 20, 123879);
    
    for (int i = 0; i < 10; i++)
      {
	std::cout << "[" << i << "] : " << random.drawSample() << std::endl;
      }
  }
};
