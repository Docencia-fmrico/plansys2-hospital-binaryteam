// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveController : public rclcpp::Node
{
public:
  MoveController()
  : rclcpp::Node("move_controller"), state_(PROBLEM1), last_problem_(PROBLEM2)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"hall", "room"});
    problem_expert_->addInstance(plansys2::Instance{"room1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"room2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"doormat11", "room"});
    problem_expert_->addInstance(plansys2::Instance{"doormat12", "room"});
    problem_expert_->addInstance(plansys2::Instance{"doormat21", "room"});
    problem_expert_->addInstance(plansys2::Instance{"doormat22", "room"});

    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"medicines", "stuff"});

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat11 room1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected room1 doormat11)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat12 hall)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected hall doormat12)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat11 doormat12)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat12 doormat11)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat21 room2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected room2 doormat21)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat22 hall)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected hall doormat22)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat21 doormat22)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat22 doormat21)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat12 doormat22)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected doormat22 doormat12)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 hall)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(stuff_at medicines hall)"));
  }

  void step() { 

    bool new_plan = false;

    switch (state_) {

      case WORKING:
      {
        // Lets see feedback
        auto my_feedback = executor_client_->getFeedBack();
        
        for (const auto & action_feedback : my_feedback.action_execution_status) {
          if (action_feedback.status == 2) {
            std::cout << action_feedback.message_status << " " <<
              action_feedback.completion * 100.0 << "%" << std::endl;
          } 
        }

        // Check if has finished the plan
        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;
          } else {
            std::cout << "Failure finished " << std::endl;
          }

          if (last_problem_ == PROBLEM1) {
            state_ = PROBLEM2;
          } else {
            state_ = PROBLEM1;
          }
        }
      }
      break;
    
      case PROBLEM1:
      {
        problem_expert_->clearGoal();
        problem_expert_->setGoal(plansys2::Goal("(and(stuff_at medicines room1) (robot_at r2d2 room2)))"));
        new_plan = true;
        last_problem_ = PROBLEM1;

      }
      break;
      
      case PROBLEM2:
      {
        problem_expert_->clearGoal();
        problem_expert_->setGoal(plansys2::Goal("(and(stuff_at medicines hall) (robot_at r2d2 hall)))"));
        new_plan = true;
        last_problem_ = PROBLEM2;
      }
      break;
    }


    if (new_plan) {

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (!plan.has_value()) {
        std::cout << "[ERROR] Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        return;
      }

      if (executor_client_->start_plan_execution(plan.value())) {
        std::cout << "Plan started succesfully!" << std::endl;
      } else {
        std::cout << "[ERROR] Plan failed at start!" << std::endl;
      }
      state_ = WORKING;
    }

  }

private:
  typedef enum {WORKING, PROBLEM1, PROBLEM2} StateType;
  StateType state_;
  StateType last_problem_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
