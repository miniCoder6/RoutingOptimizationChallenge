#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <chrono>
#include <string>
#include <iomanip>
#include <map>
#include <fstream>
#include <sstream>
#include <set>

using namespace std;

// =================== CONFIGURATION ===================

const double EARTH_RADIUS_KM = 6371.0;
const double PI = 3.14159265358979323846;
const double INF = 1e15;

// These will be loaded from metadata.csv
double OBJ_COST_WEIGHT = 0.7;
double OBJ_TIME_WEIGHT = 0.3;
int PRIORITY_DELAY[6] = {30, 5, 10, 15, 20, 30}; // index 0 = default, 1-5 = priorities

string trim(const string &str)
{
    string s = str;
    s.erase(remove(s.begin(), s.end(), '\r'), s.end());
    s.erase(remove(s.begin(), s.end(), '\n'), s.end());

    size_t first = s.find_first_not_of(" \t");
    if (first == string::npos)
        return "";
    size_t last = s.find_last_not_of(" \t");
    return s.substr(first, (last - first + 1));
}

int getPriorityDelay(int priority)
{
    if (priority >= 1 && priority <= 5)
        return PRIORITY_DELAY[priority];
    return PRIORITY_DELAY[0];
}

// Penalty multiplier for time window violations (from reference snippet)
const double INFEASIBILITY_PENALTY = 10000.0;

// =================== ENUMS & HELPERS ===================

enum VehicleCat
{
    NORMAL = 0,
    PREMIUM = 1
};

int timeToMinutes(const string &t)
{
    int hh = stoi(t.substr(0, 2));
    int mm = stoi(t.substr(3, 2));
    return hh * 60 + mm;
}

string minToTime(int m)
{
    int hh = m / 60;
    int mm = m % 60;
    string h_s = (hh < 10 ? "0" : "") + to_string(hh);
    string m_s = (mm < 10 ? "0" : "") + to_string(mm);
    return h_s + ":" + m_s;
}
// =================== METADATA READER ===================

void readMetadata(const string &filename)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "[WARN] Cannot open metadata file " << filename << ", using defaults." << endl;
        return;
    }

    string line;
    getline(file, line); // Skip header

    map<string, string> meta;
    while (getline(file, line))
    {
        stringstream ss(line);
        string key, value;
        getline(ss, key, ',');
        getline(ss, value, ',');
        meta[trim(key)] = trim(value);
    }
    file.close();

    if (meta.count("priority_1_max_delay_min"))
        PRIORITY_DELAY[1] = stoi(meta["priority_1_max_delay_min"]);
    if (meta.count("priority_2_max_delay_min"))
        PRIORITY_DELAY[2] = stoi(meta["priority_2_max_delay_min"]);
    if (meta.count("priority_3_max_delay_min"))
        PRIORITY_DELAY[3] = stoi(meta["priority_3_max_delay_min"]);
    if (meta.count("priority_4_max_delay_min"))
        PRIORITY_DELAY[4] = stoi(meta["priority_4_max_delay_min"]);
    if (meta.count("priority_5_max_delay_min"))
        PRIORITY_DELAY[5] = stoi(meta["priority_5_max_delay_min"]);
    PRIORITY_DELAY[0] = PRIORITY_DELAY[5]; // default = worst case

    if (meta.count("objective_cost_weight"))
        OBJ_COST_WEIGHT = stod(meta["objective_cost_weight"]);
    if (meta.count("objective_time_weight"))
        OBJ_TIME_WEIGHT = stod(meta["objective_time_weight"]);

    cout << "[INFO] Metadata loaded from " << filename << endl;
    cout << "  Priority delays: ";
    for (int i = 1; i <= 5; i++)
        cout << "P" << i << "=" << PRIORITY_DELAY[i] << " ";
    cout << endl;
    cout << "  Weights: cost=" << OBJ_COST_WEIGHT << " time=" << OBJ_TIME_WEIGHT << endl;
}

// =================== DATA STRUCTURES ===================

enum ShareType
{
    SINGLE = 1,
    DOUBLE = 2,
    TRIPLE = 3,
    ANY = 99
};

struct Person
{
    int id;
    string original_id;
    int priority;
    double p_lat, p_lng;
    double d_lat, d_lng;
    int early_pickup;
    int late_drop;
    VehicleCat pref_vehicle;
    int load;
    ShareType max_sharing;
};

struct Driver
{
    int id;
    string original_id;
    int capacity;
    double cost_per_km;
    double speed_kmph;
    double start_lat, start_lng;
    int start_time;
    VehicleCat category;
    string type_str;
};

// =================== DISTANCE MATRIX ===================

vector<vector<double>> distMatrix;

double calculateHaversine(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * (PI / 180.0);
    double dlon = (lon2 - lon1) * (PI / 180.0);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * (PI / 180.0)) * cos(lat2 * (PI / 180.0)) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_KM * c;
}

void precomputeDistances(const vector<Person> &persons, const vector<Driver> &drivers)
{
    int N = persons.size();
    int M = drivers.size();
    int totalPoints = N + 1 + M;

    distMatrix.assign(totalPoints, vector<double>(totalPoints, 0.0));

    vector<pair<double, double>> coords;
    for (const auto &p : persons)
        coords.push_back({p.p_lat, p.p_lng});
    coords.push_back({persons[0].d_lat, persons[0].d_lng});
    for (const auto &d : drivers)
        coords.push_back({d.start_lat, d.start_lng});

    for (int i = 0; i < totalPoints; i++)
    {
        for (int j = 0; j < totalPoints; j++)
        {
            if (i == j)
                distMatrix[i][j] = 0;
            else
            {
                distMatrix[i][j] = calculateHaversine(coords[i].first, coords[i].second,
                                                      coords[j].first, coords[j].second);
            }
        }
    }
}

double getDistPP(int i, int j) { return distMatrix[i][j]; }
double getDistPO(int i, int n_emp) { return distMatrix[i][n_emp]; }
double getDistVP(int k, int i, int n_emp) { return distMatrix[n_emp + 1 + k][i]; }
double getDistOP(int i, int n_emp) { return distMatrix[n_emp][i]; }

// =================== ALGORITHM CLASSES ===================

class Chromosome
{
public:
    vector<int> giantTour;
    double fitness;
    int numVehiclesUsed;

    struct Trip
    {
        int vehicleIdx;
        vector<int> customers;
        int startTime;
        int finishTime;
        double cost;
        double distance;
        double passengerRideTime;
    };
    vector<Trip> schedule;

    Chromosome(int n) : giantTour(n), fitness(INF), numVehiclesUsed(0)
    {
        iota(giantTour.begin(), giantTour.end(), 0);
    }
};

// =================== SPLIT PROCEDURE ===================

struct Label
{
    double cost;
    int pred;
    int vehicleIdx;
    int finishTime;
    vector<int> driverAvailTimes;
};

void splitProcedure(Chromosome &chromo, const vector<Person> &persons, const vector<Driver> &drivers)
{
    int N = persons.size();
    int M = drivers.size();
    // int OfficeIdx = N;

    vector<Label> V(N + 1);

    V[0].cost = 0;
    V[0].pred = -1;
    V[0].driverAvailTimes.resize(M);
    for (int k = 0; k < M; k++)
        V[0].driverAvailTimes[k] = drivers[k].start_time;

    for (int i = 1; i <= N; i++)
    {
        V[i].cost = INF;
        V[i].driverAvailTimes.assign(M, 0);
    }

    for (int i = 0; i < N; i++)
    {
        if (V[i].cost >= INF)
            continue;

        double currentRouteDist = 0;
        int currentLoad = 0;
        int maxCap = 0;
        for (const auto &d : drivers)
            maxCap = max(maxCap, d.capacity);

        for (int j = i + 1; j <= N && (j - i) <= maxCap; j++)
        {
            int custIdx = chromo.giantTour[j - 1];
            int prevCustIdx = (j - 1 == i) ? -1 : chromo.giantTour[j - 2];

            currentLoad += persons[custIdx].load;
            int currentGroupSize = (j - i);
            bool sharingViolation = false;

            for (int p = i; p < j; p++)
            {
                int memberIdx = chromo.giantTour[p];
                if (currentGroupSize > persons[memberIdx].max_sharing)
                {
                    sharingViolation = true;
                    break;
                }
            }

            if (sharingViolation)
                break;

            if (prevCustIdx == -1)
            {
                currentRouteDist = 0;
            }
            else
            {
                currentRouteDist += getDistPP(prevCustIdx, custIdx);
            }

            double distToOffice = getDistPO(custIdx, N);

            double bestTripCost = INF;
            int bestVehicle = -1;
            int bestFinishTime = -1;

            for (int k = 0; k < M; k++)
            {
                const Driver &d = drivers[k];

                if (d.capacity < currentLoad)
                    continue;

                bool prefFail = false;
                for (int p = i; p < j; p++)
                {
                    if (persons[chromo.giantTour[p]].pref_vehicle == PREMIUM && d.category != PREMIUM)
                    {
                        prefFail = true;
                        break;
                    }
                }
                if (prefFail)
                    continue;

                int firstCust = chromo.giantTour[i];
                int availTime = V[i].driverAvailTimes[k];
                double distStartToFirst = 0;

                if (availTime == d.start_time)
                {
                    distStartToFirst = getDistVP(k, firstCust, N);
                }
                else
                {
                    distStartToFirst = getDistOP(firstCust, N);
                }

                double totalDist = distStartToFirst + currentRouteDist + distToOffice;

                double timeToFirst = (distStartToFirst / d.speed_kmph) * 60;
                int arrivalAtNode = availTime + ceil(timeToFirst);
                int currentVisTime = max(arrivalAtNode, persons[firstCust].early_pickup);

                vector<int> pickupTimes;
                pickupTimes.push_back(currentVisTime);

                int tempPrev = firstCust;

                for (int p = i + 1; p < j; p++)
                {
                    int pIdx = chromo.giantTour[p];
                    double legDist = getDistPP(tempPrev, pIdx);
                    int legTime = ceil((legDist / d.speed_kmph) * 60);

                    int arrival = currentVisTime + legTime;
                    int startService = max(arrival, persons[pIdx].early_pickup);

                    pickupTimes.push_back(startService);
                    currentVisTime = startService;
                    tempPrev = pIdx;
                }

                int travelToOffice = ceil((distToOffice / d.speed_kmph) * 60);
                int arrivalAtOffice = currentVisTime + travelToOffice;

                // ===== INFEASIBILITY PENALTY INSTEAD OF HARD REJECTION =====
                double penalty = 0.0;

                for (int p = i; p < j; p++)
                {
                    int pIdx = chromo.giantTour[p];
                    int idxInGroup = p - i;
                    int actualPickup = pickupTimes[idxInGroup];

                    // Penalty for late drop-off violation
                    if (arrivalAtOffice > persons[pIdx].late_drop)
                    {
                        long long violation = arrivalAtOffice - persons[pIdx].late_drop;
                        penalty += INFEASIBILITY_PENALTY * violation;
                    }

                    // Penalty for priority-based max delay violation
                    int actualRideTime = arrivalAtOffice - actualPickup;
                    double directDist = getDistPO(pIdx, N);
                    int directTime = ceil((directDist / d.speed_kmph) * 60);
                    int delay = actualRideTime - directTime;
                    int allowedDelay = getPriorityDelay(persons[pIdx].priority);

                    if (delay > allowedDelay)
                    {
                        long long violation = delay - allowedDelay;
                        penalty += INFEASIBILITY_PENALTY * violation;
                    }
                }

                double totalPassengerRideTime = 0;
                for (int p = i; p < j; p++)
                {
                    // int pIdx = chromo.giantTour[p];
                    int idxInGroup = p - i;
                    int actualPickup = pickupTimes[idxInGroup];
                    int rideTime = arrivalAtOffice - actualPickup;
                    totalPassengerRideTime += rideTime;
                }

                double monetaryCost = totalDist * d.cost_per_km;
                double cost = (monetaryCost * OBJ_COST_WEIGHT) + (totalPassengerRideTime * OBJ_TIME_WEIGHT) + penalty;

                if (cost < bestTripCost)
                {
                    bestTripCost = cost;
                    bestVehicle = k;
                    bestFinishTime = arrivalAtOffice;
                }
            }

            if (bestVehicle != -1)
            {
                double newCost = V[i].cost + bestTripCost;
                if (newCost < V[j].cost)
                {
                    V[j].cost = newCost;
                    V[j].pred = i;
                    V[j].vehicleIdx = bestVehicle;
                    V[j].finishTime = bestFinishTime;
                    V[j].driverAvailTimes = V[i].driverAvailTimes;
                    V[j].driverAvailTimes[bestVehicle] = bestFinishTime;
                }
            }
        }
    }

    chromo.fitness = V[N].cost;

    if (chromo.fitness >= INF)
        return;

    int curr = N;
    map<int, bool> usedVehicles;

    while (curr > 0)
    {
        int prev = V[curr].pred;
        int vIdx = V[curr].vehicleIdx;

        Chromosome::Trip trip;
        trip.vehicleIdx = vIdx;
        trip.finishTime = V[curr].finishTime;

        for (int k = prev; k < curr; k++)
        {
            trip.customers.push_back(chromo.giantTour[k]);
        }

        chromo.schedule.push_back(trip);
        usedVehicles[vIdx] = true;
        curr = prev;
    }

    chromo.numVehiclesUsed = usedVehicles.size();
    reverse(chromo.schedule.begin(), chromo.schedule.end());
}

// =================== GENETIC OPERATORS ===================

Chromosome crossover(const Chromosome &p1, const Chromosome &p2, mt19937 &rng)
{
    int n = p1.giantTour.size();
    Chromosome child(n);
    vector<bool> present(n, false);

    int start = rng() % n;
    int end = rng() % n;
    if (start > end)
        swap(start, end);

    for (int i = start; i <= end; i++)
    {
        child.giantTour[i] = p1.giantTour[i];
        present[p1.giantTour[i]] = true;
    }

    int curr = (end + 1) % n;
    int p2_idx = (end + 1) % n;

    while (curr != start)
    {
        int gene = p2.giantTour[p2_idx];
        if (!present[gene])
        {
            child.giantTour[curr] = gene;
            curr = (curr + 1) % n;
        }
        p2_idx = (p2_idx + 1) % n;
    }
    return child;
}

void mutate(Chromosome &c, mt19937 &rng)
{
    int n = c.giantTour.size();
    int i = rng() % n;
    int j = rng() % n;
    swap(c.giantTour[i], c.giantTour[j]);
}

// =================== DETAILED METRICS CALCULATION ===================

struct SolutionMetrics
{
    double totalDistance;
    double totalDuration;
    double totalPassengerRideTime;
    double monetaryCost;
    double weightedObjective;
};

SolutionMetrics calculateDetailedMetrics(const Chromosome &best, const vector<Person> &persons, const vector<Driver> &drivers)
{
    SolutionMetrics metrics = {0, 0, 0, 0, 0};
    int N = persons.size();

    map<int, vector<Chromosome::Trip>> driverTrips;
    for (const auto &t : best.schedule)
    {
        driverTrips[t.vehicleIdx].push_back(t);
    }

    for (auto &dt : driverTrips)
    {
        int vIdx = dt.first;
        const Driver &d = drivers[vIdx];

        int vehicleStartTime = d.start_time;
        int currentTime = vehicleStartTime;

        for (int i = 0; i < (int)dt.second.size(); i++)
        {
            auto &trip = dt.second[i];
            int firstCust = trip.customers[0];
            double legDist = 0;

            if (i == 0)
            {
                legDist = getDistVP(vIdx, firstCust, N);
            }
            else
            {
                legDist = getDistOP(firstCust, N);
            }

            int travelTime = ceil((legDist / d.speed_kmph) * 60);
            currentTime = max(currentTime + travelTime, persons[firstCust].early_pickup);

            metrics.totalDistance += legDist;
            int prevCust = firstCust;

            vector<int> pickupTimes;
            pickupTimes.push_back(currentTime);

            for (size_t k = 1; k < trip.customers.size(); k++)
            {
                int cIdx = trip.customers[k];
                double dist = getDistPP(prevCust, cIdx);
                int travel = ceil((dist / d.speed_kmph) * 60);
                currentTime = max(currentTime + travel, persons[cIdx].early_pickup);

                pickupTimes.push_back(currentTime);
                metrics.totalDistance += dist;
                prevCust = cIdx;
            }

            double toOffice = getDistPO(prevCust, N);
            int travelOffice = ceil((toOffice / d.speed_kmph) * 60);
            int arrivalAtOffice = currentTime + travelOffice;
            metrics.totalDistance += toOffice;

            for (size_t k = 0; k < trip.customers.size(); k++)
            {
                int rideTime = arrivalAtOffice - pickupTimes[k];
                metrics.totalPassengerRideTime += rideTime;
            }

            currentTime = arrivalAtOffice;
        }

        metrics.totalDuration += (currentTime - vehicleStartTime);
    }

    metrics.monetaryCost = metrics.totalDistance * 0.0;
    for (auto &dt : driverTrips)
    {
        const Driver &d = drivers[dt.first];
        double vehDist = 0;
        // int currentTime = d.start_time;

        for (int i = 0; i < dt.second.size(); i++)
        {
            auto &trip = dt.second[i];
            int firstCust = trip.customers[0];
            double legDist = (i == 0) ? getDistVP(dt.first, firstCust, N) : getDistOP(firstCust, N);
            vehDist += legDist;

            int prevCust = firstCust;
            for (size_t k = 1; k < trip.customers.size(); k++)
            {
                int cIdx = trip.customers[k];
                vehDist += getDistPP(prevCust, cIdx);
                prevCust = cIdx;
            }
            vehDist += getDistPO(prevCust, N);
        }
        metrics.monetaryCost += vehDist * d.cost_per_km;
    }

    metrics.weightedObjective = (metrics.monetaryCost * OBJ_COST_WEIGHT) +
                                (metrics.totalPassengerRideTime * OBJ_TIME_WEIGHT);

    return metrics;
}

// =================== CSV READING FUNCTIONS ===================

vector<Driver> readVehicleCSV(const string &filename)
{
    vector<Driver> drivers;
    ifstream file(filename);

    if (!file.is_open())
    {
        cerr << "Error: Cannot open file " << filename << endl;
        return drivers;
    }

    string line;
    if (line.back() == '\r')
        line.pop_back();
    getline(file, line); // Skip header

    int id = 0;
    while (getline(file, line))
    {
        stringstream ss(line);
        if (line.back() == '\r')
            line.pop_back();
        string vehicle_id, fuel_type, vehicle_mode, seating_str, cost_str, speed_str, loc_x_str, loc_y_str, avail_time_str, category_str;

        getline(ss, vehicle_id, ',');
        getline(ss, fuel_type, ',');
        getline(ss, vehicle_mode, ',');
        getline(ss, seating_str, ',');
        getline(ss, cost_str, ',');
        getline(ss, speed_str, ',');
        getline(ss, loc_x_str, ',');
        getline(ss, loc_y_str, ',');
        getline(ss, avail_time_str, ',');
        getline(ss, category_str, ',');

        cout << vehicle_id << " " << category_str << endl;

        Driver d;
        d.id = id++;
        d.original_id = trim(vehicle_id);
        d.capacity = stoi(trim(seating_str));
        d.cost_per_km = stod(trim(cost_str));
        d.speed_kmph = stod(trim(speed_str));
        d.start_lat = stod(trim(loc_x_str));
        d.start_lng = stod(trim(loc_y_str));
        d.start_time = timeToMinutes(trim(avail_time_str));

        string cat = trim(category_str);
        d.category = (cat == "premium") ? PREMIUM : NORMAL;
        d.type_str = trim(fuel_type) + "/" + trim(vehicle_mode);

        drivers.push_back(d);
    }

    file.close();
    cout << "[INFO] Loaded " << drivers.size() << " vehicles from " << filename << endl;
    return drivers;
}

vector<Person> readEmployeeCSV(const string &filename)
{
    vector<Person> persons;
    ifstream file(filename);

    if (!file.is_open())
    {
        cerr << "Error: Cannot open file " << filename << endl;
        return persons;
    }

    string line;
    getline(file, line); // Skip header

    int id = 0;
    while (getline(file, line))
    {
        stringstream ss(line);
        string emp_id, priority_str, pickup_x_str, pickup_y_str, dest_x_str, dest_y_str, time_start_str, time_end_str, veh_pref_str, share_pref_str;

        getline(ss, emp_id, ',');
        getline(ss, priority_str, ',');
        getline(ss, pickup_x_str, ',');
        getline(ss, pickup_y_str, ',');
        getline(ss, dest_x_str, ',');
        getline(ss, dest_y_str, ',');
        getline(ss, time_start_str, ',');
        getline(ss, time_end_str, ',');
        getline(ss, veh_pref_str, ',');
        getline(ss, share_pref_str);

        cout << emp_id << " " << share_pref_str << endl;

        Person p;
        p.id = id++;
        p.original_id = trim(emp_id);
        p.priority = stoi(trim(priority_str));
        p.p_lat = stod(trim(pickup_x_str));
        p.p_lng = stod(trim(pickup_y_str));
        p.d_lat = stod(trim(dest_x_str));
        p.d_lng = stod(trim(dest_y_str));
        p.early_pickup = timeToMinutes(trim(time_start_str));
        p.late_drop = timeToMinutes(trim(time_end_str));

        string veh_pref = trim(veh_pref_str);
        p.pref_vehicle = (veh_pref == "premium") ? PREMIUM : NORMAL;

        string share_pref = trim(share_pref_str);
        if (share_pref == "single")
            p.max_sharing = SINGLE;
        else if (share_pref == "double")
            p.max_sharing = DOUBLE;
        else if (share_pref == "triple")
            p.max_sharing = TRIPLE;
        else
            p.max_sharing = ANY;

        p.load = 1;

        persons.push_back(p);
    }

    file.close();
    cout << "[INFO] Loaded " << persons.size() << " employees from " << filename << endl;
    return persons;
}

map<string, int> buildEmployeeIdMap(const vector<Person> &persons)
{
    map<string, int> empMap;
    for (const auto &p : persons)
    {
        empMap[p.original_id] = p.id;
    }
    return empMap;
}

map<string, int> buildVehicleIdMap(const vector<Driver> &drivers)
{
    map<string, int> vehMap;
    for (const auto &d : drivers)
    {
        vehMap[d.original_id] = d.id;
    }
    return vehMap;
}

Chromosome readCSVSolution(const string &filename, const vector<Person> &persons, const vector<Driver> &drivers)
{
    Chromosome chromo(persons.size());

    map<string, int> empMap = buildEmployeeIdMap(persons);
    map<string, int> vehMap = buildVehicleIdMap(drivers);

    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Cannot open file " << filename << endl;
        return chromo;
    }

    string line;
    getline(file, line); // Skip header

    vector<int> tour;
    set<int> seen;

    while (getline(file, line))
    {
        stringstream ss(line);
        string vehicle_id, category, employee_id, pickup_time, drop_time;

        getline(ss, vehicle_id, ',');
        getline(ss, category, ',');
        getline(ss, employee_id, ',');
        getline(ss, pickup_time, ',');
        getline(ss, drop_time, ',');

        employee_id = trim(employee_id);

        if (empMap.find(employee_id) != empMap.end())
        {
            int empIdx = empMap[employee_id];
            if (seen.find(empIdx) == seen.end())
            {
                tour.push_back(empIdx);
                seen.insert(empIdx);
            }
        }
    }

    file.close();

    // Add any missing employees
    for (const auto &p : persons)
    {
        if (seen.find(p.id) == seen.end())
        {
            tour.push_back(p.id);
        }
    }

    chromo.giantTour = tour;
    return chromo;
}

// =================== CSV OUTPUT FUNCTIONS ===================

void writeCSVOutput(const string &filename, const Chromosome &solution, const vector<Person> &persons, const vector<Driver> &drivers)
{
    ofstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Cannot create output file " << filename << endl;
        return;
    }

    file << "vehicle_id,category,employee_id,pickup_time,drop_time\n";

    int N = persons.size();

    map<int, vector<Chromosome::Trip>> driverTrips;
    for (const auto &t : solution.schedule)
    {
        driverTrips[t.vehicleIdx].push_back(t);
    }

    for (auto &dt : driverTrips)
    {
        int vIdx = dt.first;
        const Driver &d = drivers[vIdx];

        int currentTime = d.start_time;

        for (int i = 0; i < (int)dt.second.size(); i++)
        {
            auto &trip = dt.second[i];
            int firstCust = trip.customers[0];
            double legDist = 0;

            if (i == 0)
            {
                legDist = getDistVP(vIdx, firstCust, N);
            }
            else
            {
                legDist = getDistOP(firstCust, N);
            }

            int travelTime = ceil((legDist / d.speed_kmph) * 60);
            currentTime = max(currentTime + travelTime, persons[firstCust].early_pickup);

            vector<int> pickupTimes;
            pickupTimes.push_back(currentTime);
            int prevCust = firstCust;

            for (size_t k = 1; k < trip.customers.size(); k++)
            {
                int cIdx = trip.customers[k];
                double dist = getDistPP(prevCust, cIdx);
                int travel = ceil((dist / d.speed_kmph) * 60);
                currentTime = max(currentTime + travel, persons[cIdx].early_pickup);

                pickupTimes.push_back(currentTime);
                prevCust = cIdx;
            }

            double toOffice = getDistPO(prevCust, N);
            int travelOffice = ceil((toOffice / d.speed_kmph) * 60);
            int arrivalAtOffice = currentTime + travelOffice;

            for (size_t k = 0; k < trip.customers.size(); k++)
            {
                int empIdx = trip.customers[k];
                file << d.original_id << ","
                     << (d.category == PREMIUM ? "Premium" : "Normal") << ","
                     << persons[empIdx].original_id << ","
                     << minToTime(pickupTimes[k]) << ","
                     << minToTime(arrivalAtOffice) << "\n";
            }

            currentTime = arrivalAtOffice;
        }
    }

    file.close();
}

// =================== MAIN DRIVER ===================

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <test_case_folder_path>" << endl;
        cerr << "Example: " << argv[0] << " /home/yash/kriti/test_cases/TC_01" << endl;
        return 1;
    }

    string basePath = argv[1];
    // Ensure trailing slash
    if (basePath.back() != '/')
        basePath += '/';

    // =================== READ METADATA ===================

    cout << "[INFO] Reading metadata..." << endl;
    readMetadata(basePath + "metadata.csv");

    // =================== READ VEHICLE & EMPLOYEE DATA ===================

    cout << "[INFO] Reading input data from CSV files..." << endl;
    vector<Driver> drivers = readVehicleCSV(basePath + "vehicles.csv");
    vector<Person> persons = readEmployeeCSV(basePath + "employees.csv");

    if (drivers.empty() || persons.empty())
    {
        cerr << "Error: Failed to load vehicle or employee data!" << endl;
        return 1;
    }

    cout << "[INFO] Precomputing distances..." << endl;
    precomputeDistances(persons, drivers);

    // =================== READ 5 ALGORITHM OUTPUTS AS INITIAL POPULATION ===================

    vector<string> subfolders = {
        "ALNS",
        "Branch-And-Cut",
        "Clustering-Routing-DP-Solver",
        "Heterogeneous_DARP",
        "Variable_Neighbourhood_Search",
        "god"};

    cout << "[INFO] Reading initial population from algorithm subfolder outputs..." << endl;
    vector<Chromosome> population;

    for (const auto &folder : subfolders)
    {
        string filepath = basePath + folder + "/output_vehicle.csv";
        cout << "[INFO] Reading: " << filepath << endl;

        ifstream testFile(filepath);
        if (!testFile.is_open())
        {
            cerr << "[WARN] Skipping " << filepath << " (file not found)" << endl;
            continue;
        }
        testFile.close();

        Chromosome c = readCSVSolution(filepath, persons, drivers);
        splitProcedure(c, persons, drivers);
        population.push_back(c);
        cout << "  - Fitness: " << fixed << setprecision(2) << c.fitness << endl;
    }

    if (population.empty())
    {
        cerr << "Error: No initial solutions could be loaded!" << endl;
        return 1;
    }

    int POP_SIZE = population.size();
    int GENERATIONS = 200;
    mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

    cout << "[INFO] Starting GA with population size " << POP_SIZE << " for " << GENERATIONS << " generations..." << endl;

    auto start_time = chrono::high_resolution_clock::now();

    for (int gen = 0; gen < GENERATIONS; gen++)
    {
        sort(population.begin(), population.end(), [](const Chromosome &a, const Chromosome &b)
             { return a.fitness < b.fitness; });

        if (gen % 50 == 0)
        {
            cout << "Gen " << gen << " | Best Weighted Cost: " << fixed << setprecision(2)
                 << population[0].fitness << " | Vehicles: " << population[0].numVehiclesUsed << endl;
        }

        vector<Chromosome> nextPop;
        nextPop.push_back(population[0]); // Keep best

        while ((int)nextPop.size() < POP_SIZE)
        {
            int t1 = rng() % POP_SIZE;
            int t2 = rng() % POP_SIZE;

            Chromosome child = crossover(population[t1], population[t2], rng);
            if (rng() % 100 < 15)
                mutate(child, rng);

            splitProcedure(child, persons, drivers);
            nextPop.push_back(child);
        }
        population = nextPop;
    }

    auto end_time = chrono::high_resolution_clock::now();
    double duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count() / 1000.0;

    sort(population.begin(), population.end(), [](const Chromosome &a, const Chromosome &b)
         { return a.fitness < b.fitness; });
    Chromosome best = population[0];

    // =================== OUTPUT TO CSV ===================

    cout << "\n[INFO] Writing final solution to CSV..." << endl;
    writeCSVOutput(basePath + "mem/final_output_vehicle.csv", best, persons, drivers);
    cout << "[INFO] Output written to: " << basePath << "/final_output_vehicle.csv" << endl;

    // =================== ENHANCED OUTPUT WITH METRICS ===================

    cout << "\n"
         << string(80, '=') << "\n";
    cout << "                    FINAL METRICS BREAKDOWN\n";
    cout << string(80, '=') << "\n\n";

    if (best.fitness >= INF)
    {
        cout << "NO FEASIBLE SOLUTION FOUND!" << endl;
    }
    else
    {
        SolutionMetrics metrics = calculateDetailedMetrics(best, persons, drivers);

        cout << "Total Distance:                   " << fixed << setprecision(2)
             << metrics.totalDistance << " km\n";
        cout << "Total Time (Duration):            " << fixed << setprecision(0)
             << metrics.totalDuration << " min\n";
        cout << "Total Passenger Ride Time:        " << fixed << setprecision(0)
             << metrics.totalPassengerRideTime << " min\n";
        cout << "TOTAL COST:                       " << fixed << setprecision(2)
             << metrics.monetaryCost << "\n\n";

        cout << string(80, '=') << "\n";
        cout << "WEIGHTED OBJECTIVE FUNCTION:\n";
        cout << "  (" << fixed << setprecision(2) << metrics.monetaryCost << " * "
             << OBJ_COST_WEIGHT << ") + (" << metrics.totalPassengerRideTime << " * "
             << OBJ_TIME_WEIGHT << ") = " << metrics.weightedObjective << "\n";
        cout << string(80, '=') << "\n\n";

        cout << "Execution Time:                   " << fixed << setprecision(3)
             << duration << " seconds\n";
        cout << "Vehicles Used:                    " << best.numVehiclesUsed << "\n\n";

        // =================== DETAILED ROUTE OUTPUT ===================

        map<int, vector<Chromosome::Trip>> driverTrips;
        for (const auto &t : best.schedule)
        {
            driverTrips[t.vehicleIdx].push_back(t);
        }

        cout << string(80, '=') << "\n";
        cout << "                    DETAILED ROUTE SCHEDULE\n";
        cout << string(80, '=') << "\n";

        for (auto &dt : driverTrips)
        {
            int vIdx = dt.first;
            const Driver &d = drivers[vIdx];
            cout << "\nVehicle " << d.original_id << " (" << d.type_str << ") "
                 << "- Capacity: " << d.capacity << " | Cost/km: Rs" << d.cost_per_km << endl;

            int currentTime = d.start_time;
            double vehicleTotalDist = 0;

            for (int i = 0; i < (int)dt.second.size(); i++)
            {
                auto &trip = dt.second[i];
                cout << "  Trip " << i + 1 << " (" << trip.customers.size() << " passenger"
                     << (trip.customers.size() > 1 ? "s" : "") << "): ";

                int firstCust = trip.customers[0];
                double legDist = 0;

                if (i == 0)
                {
                    legDist = getDistVP(vIdx, firstCust, persons.size());
                    cout << "[Base -> ";
                }
                else
                {
                    legDist = getDistOP(firstCust, persons.size());
                    cout << "[Office -> ";
                }

                int travelTime = ceil((legDist / d.speed_kmph) * 60);
                currentTime = max(currentTime + travelTime, persons[firstCust].early_pickup);

                cout << persons[firstCust].original_id << " @" << minToTime(currentTime) << "]";

                vehicleTotalDist += legDist;
                int prevCust = firstCust;

                for (size_t k = 1; k < trip.customers.size(); k++)
                {
                    int cIdx = trip.customers[k];
                    double dist = getDistPP(prevCust, cIdx);
                    int travel = ceil((dist / d.speed_kmph) * 60);
                    currentTime = max(currentTime + travel, persons[cIdx].early_pickup);

                    cout << " -> " << persons[cIdx].original_id << " @" << minToTime(currentTime);
                    vehicleTotalDist += dist;
                    prevCust = cIdx;
                }

                double toOffice = getDistPO(prevCust, persons.size());
                int travelOffice = ceil((toOffice / d.speed_kmph) * 60);
                currentTime += travelOffice;
                vehicleTotalDist += toOffice;

                cout << " -> OFFICE (Drop @" << minToTime(currentTime) << ")" << endl;
            }
            cout << "  Total Distance: " << fixed << setprecision(2) << vehicleTotalDist
                 << " km | Cost: Rs" << (vehicleTotalDist * d.cost_per_km) << endl;
        }

        cout << "\n"
             << string(80, '=') << "\n";
    }

    cin.get();
    return 0;
}
