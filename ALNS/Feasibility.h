#pragma once
#include "Route.h"
#include "Vehicle.h"
#include "Employee.h"

#include "Distance.h"
#include <vector>
#include <algorithm>
#include <cmath>

inline bool routeFeasible(const Route& r,
                          const Vehicle& v,
                          const std::vector<Employee>& emp){

    if (r.seq.empty()) return true;

    double t = v.startTime;
    double cx = v.x;
    double cy = v.y;
    std::vector<int> batch; 
    
    auto checkBatch = [&](const std::vector<int>& b, int nextId) -> bool {
        if (b.size() + 1 > v.seatCap) return false;
        
        const auto& nextE = emp[nextId];
        int newSize = b.size() + 1;
        if (nextE.sharePref < newSize) return false;
        for (int pid : b) {
            if (emp[pid].sharePref < newSize) return false;
        }
        
        return true;
    };

    for (size_t i = 0; i < r.seq.size(); ++i) {
        int eId = r.seq[i];
        const auto& e = emp[eId];


        bool fits = checkBatch(batch, eId);

        double travel = distKm(cx, cy, e.x, e.y) / v.speed * 60.0;
        
        double dKm = distKm(cx, cy, e.x, e.y); 
        double travelMin = (dKm / v.speed) * 60.0;
        
        double arrival = t + travelMin;
        double startService = std::max(arrival, e.ready);
        double depart = startService; 
        double dDestKm = distKm(e.x, e.y, e.destX, e.destY);
        double timeToDest = (dDestKm / v.speed) * 60.0;
        double arrivalAtDest = depart + timeToDest;
        // Hard constraint: delay > 180 min is infeasible
        double delay = arrivalAtDest - e.due;
        if (delay > 180.0) return false; 
        if (fits) {
             for (int bid : batch) {
                 double batchDelay = arrivalAtDest - emp[bid].due;
                 if (batchDelay > 180.0) return false;
             }
        }

        if (fits) {
            batch.push_back(eId);
            t = depart;
            cx = e.x; 
            cy = e.y;
        } else {
            if (batch.empty()) return false; 
            if (batch.empty()) return false;
            const auto& last = emp[batch.back()];
            double dOff = distKm(cx, cy, last.destX, last.destY);
            double tOff = (dOff / v.speed) * 60.0;
            t += tOff;

            if (t > v.endTime) return false;
            
            batch.clear();
            
            double dToE = distKm(last.destX, last.destY, e.x, e.y);
            t += (dToE / v.speed) * 60.0;
                       
            cx = last.destX; cy = last.destY;

            dKm = distKm(cx, cy, e.x, e.y);
            travelMin = (dKm / v.speed) * 60.0;
            arrival = t + travelMin;
            startService = std::max(arrival, e.ready);
            depart = startService + 1.0;
            
            dDestKm = distKm(e.x, e.y, e.destX, e.destY);
            timeToDest = (dDestKm / v.speed) * 60.0;
            arrivalAtDest = depart + timeToDest;

            if (arrivalAtDest > v.endTime) return false; 
            
            batch.push_back(eId);
            t = depart;
            cx = e.x; cy = e.y;
        }
    }
    
    if (!batch.empty()) {
        const auto& last = emp[batch.back()];
        double dOff = distKm(cx, cy, last.destX, last.destY);
        t += (dOff / v.speed) * 60.0;
        if (t > v.endTime) return false;
    }
    
    return true;
}

