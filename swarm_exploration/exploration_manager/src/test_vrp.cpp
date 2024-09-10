#include <cstdint>
#include <sstream>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
struct DataModel {
  const std::vector<std::vector<int64_t>> distance_matrix{
      {0, 20, 40, 50, 60},
      {20, 0, 20, 30, 40},
      {40, 20, 0, 10, 20},
      {50, 30, 10, 0, 30},
      {60, 40, 20, 30, 0},
      {60, 40, 20, 30, 0}

  };
  // const std::vector<int64_t> demands{0, 5, 5, 5, 5};
  // const std::vector<int64_t> vehicle_capacities{50, 50};
  const std::vector<std::vector<int64_t>> cost_matrices{
      {50, 5, 50, 5, 50},
      {5, 50, 5, 50, 5},
  };
  const int num_vehicles = 2;
  const RoutingIndexManager::NodeIndex depot{0};
};

void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  int64_t total_distance = 0;
  //int64_t total_load = 0;
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64_t index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64_t route_distance = 0;
    //int64_t route_load = 0;
    int64_t route_d2 = 0;
    std::stringstream route;
    while (!routing.IsEnd(index)) {
      const int node_index = manager.IndexToNode(index).value();
      //route_load += data.demands[node_index];
      route << node_index << " -> ";
      const int64_t previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance = route_distance + routing.GetArcCostForVehicle(previous_index, index,
                                                     int64_t{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Distance of the route: " << route_distance << "m";
    //LOG(INFO) << "Load of the route: " << route_load;
    LOG(INFO) << "route d2: " << route_d2 << "m";
    total_distance += route_distance;
    //total_load += route_load;
  }
  LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  //LOG(INFO) << "Total load of all routes: " << total_load;
  LOG(INFO) << "";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void VrpCapacity() {
  // Instantiate the data problem.
  DataModel data;
  
  // Create Routing Index Manager
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles, data.depot);

  LOG(INFO) << "DISTANCE MATRIX size: " << data.distance_matrix.size();

  // Create Routing Model.
  RoutingModel routing(manager);

  // Define cost of each arc for each vehicle.
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    const int64_t transit_callback_index_vehicle = routing.RegisterTransitCallback(
      [&data, &manager, vehicle_id] // Variables that should be accessible inside the lambda matrix
      (const int64_t from_index, const int64_t to_index) -> int64_t { // Function parameters
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.cost_matrices[vehicle_id][to_node];
      });
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_vehicle, vehicle_id);
  }

  // // Add Capacity constraint.
  // const int demand_callback_index = routing.RegisterUnaryTransitCallback(
  //     [&data, &manager](const int64_t from_index) -> int64_t {
  //       // Convert from routing variable Index to demand NodeIndex.
  //       const int from_node = manager.IndexToNode(from_index).value();
  //       return data.demands[from_node];
  //     });
  // routing.AddDimensionWithVehicleCapacity(
  //     demand_callback_index,    // transit callback index
  //     int64_t{0},               // null capacity slack
  //     data.vehicle_capacities,  // vehicle maximum capacities
  //     true,                     // start cumul to zero
  //     "Capacity");

  // Setting first solution heuristic.
  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  search_parameters.mutable_time_limit()->set_seconds(1);

  // Solve the problem.
  const Assignment* solution = routing.SolveWithParameters(search_parameters);

  // Print solution on console.
  PrintSolution(data, manager, routing, *solution);
}
}  // namespace operations_research

int main(int /*argc*/, char* /*argv*/[]) {

  operations_research::VrpCapacity();
  return EXIT_SUCCESS;
}


// #include <algorithm>
// #include <cstdint>
// #include <sstream>
// #include <vector>

// #include "ortools/constraint_solver/routing.h"
// #include "ortools/constraint_solver/routing_enums.pb.h"
// #include "ortools/constraint_solver/routing_index_manager.h"
// #include "ortools/constraint_solver/routing_parameters.h"

// namespace operations_research {
// struct DataModel {
//   const std::vector<std::vector<int64_t>> distance_matrix{
//       {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468,
//        776, 662},
//       {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
//        1016, 868, 1210},
//       {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130,
//        788, 1552, 754},
//       {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
//        1164, 560, 1358},
//       {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
//        1050, 674, 1244},
//       {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514,
//        1050, 708},
//       {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514,
//        1278, 480},
//       {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662,
//        742, 856},
//       {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320,
//        1084, 514},
//       {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274,
//        810, 468},
//       {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730,
//        388, 1152, 354},
//       {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308,
//        650, 274, 844},
//       {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536,
//        388, 730},
//       {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342,
//        422, 536},
//       {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342,
//        0, 764, 194},
//       {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388,
//        422, 764, 0, 798},
//       {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536,
//        194, 798, 0},
//   };
//   const int num_vehicles = 4;
//   const RoutingIndexManager::NodeIndex depot{0};
// };

// //! @brief Print the solution.
// //! @param[in] data Data of the problem.
// //! @param[in] manager Index manager used.
// //! @param[in] routing Routing solver used.
// //! @param[in] solution Solution found by the solver.
// void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
//                    const RoutingModel& routing, const Assignment& solution) {
//   int64_t max_route_distance{0};
//   std::vector<int> tour;
//   for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
//     int64_t index = routing.Start(vehicle_id);
//     LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
//     int64_t route_distance{0};
//     std::stringstream route;
//     while (!routing.IsEnd(index)) {
//       tour.push_back(manager.IndexToNode(index).value());
//       route << manager.IndexToNode(index).value() << " -> ";
//       const int64_t previous_index = index;
//       index = solution.Value(routing.NextVar(index));
//       route_distance += routing.GetArcCostForVehicle(previous_index, index,
//                                                      int64_t{vehicle_id});
//     }
//     tour.erase(tour.begin());
//     LOG(INFO) << "tour size: " << tour.size();
//     std::cout << "tour: ";
//     for(int i = 0; i < tour.size(); i++){
//       std::cout << tour[i] << ", ";
//     }
//     tour.clear();
//     std::cout << std::endl;
//     LOG(INFO) << route.str() << manager.IndexToNode(index).value();
//     LOG(INFO) << "Distance of the route: " << route_distance << "m";
//     max_route_distance = std::max(route_distance, max_route_distance);
//   }
//   LOG(INFO) << "Maximum of the route distances: " << max_route_distance << "m";
//   LOG(INFO) << "";
//   LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
// }

// void VrpGlobalSpan() {
//   // Instantiate the data problem.
//   DataModel data;

//   // Create Routing Index Manager
//   RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
//                               data.depot);

//   LOG(INFO) << "DISTANCE MATRIX size: " << data.distance_matrix.size();
//   LOG(INFO) << "NUM VEHICLES: " << data.num_vehicles;
//   LOG(INFO) << "DEPOT: " << data.depot;

//   // Create Routing Model.
//   RoutingModel routing(manager);

//   // Create and register a transit callback.
//   const int transit_callback_index = routing.RegisterTransitCallback(
//       [&data, &manager](const int64_t from_index,
//                         const int64_t to_index) -> int64_t {
//         // Convert from routing variable Index to distance matrix NodeIndex.
//         const int from_node = manager.IndexToNode(from_index).value();
//         const int to_node = manager.IndexToNode(to_index).value();
//         return data.distance_matrix[from_node][to_node];
//       });

//   // Define cost of each arc.
//   routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

//   // Add Distance constraint.
//   routing.AddDimension(transit_callback_index, 0, 3000,
//                        true,  // start cumul to zero
//                        "Distance");
//   routing.GetMutableDimension("Distance")->SetGlobalSpanCostCoefficient(100);

//   // Setting first solution heuristic.
//   RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
//   searchParameters.set_first_solution_strategy(
//       FirstSolutionStrategy::PATH_CHEAPEST_ARC);

//   // Solve the problem.
//   const Assignment* solution = routing.SolveWithParameters(searchParameters);

//   // Print solution on console.
//   if (solution != nullptr) {
//     PrintSolution(data, manager, routing, *solution);
//   } else {
//     LOG(INFO) << "No solution found.";
//   }
// }
// }  // namespace operations_research

// int main(int /*argc*/, char* /*argv*/[]) {
//   operations_research::VrpGlobalSpan();
//   return EXIT_SUCCESS;
// }