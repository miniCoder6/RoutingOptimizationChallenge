#pragma once
#include "Route.h"
#include "Vehicle.h"
#include "Employee.h"
#include <vector>

#include "CSVReader.h" // For Metadata

double routeCost(const Route&, const Vehicle&, const std::vector<Employee>&, const Metadata&);
