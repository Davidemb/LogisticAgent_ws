#include "PatrolAgent.hpp"
namespace coa
{
using namespace patrolagent;

class COA : public PatrolAgent
{
public:
  virtual void run();
  virtual void onGoalComplete();
  virtual int compute_next_vertex();
};
}  // cycle opt agent

#include "impl/CycleOptAgent.i.hpp"