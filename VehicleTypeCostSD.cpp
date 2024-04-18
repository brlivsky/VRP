/*
 * Params set are different
 * Vehicle type, vehicle cost, vehicle capacity based heuristics
 * Reads a single text file.
 * PD pairs are sorted based on largest round trip distance and earliest deadline and used only least cost
 * Timestamp
 */

#include <iostream>
#include <unordered_map>
#include <iomanip>
#include <climits>
#include <bits/stdc++.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string>
#include <cmath>
#include <time.h>
#define SDLIMIT 400

using namespace std;

int nDemandIdx = 2;
int nStartTimeIdx = 3;
int nEndTimeIdx = 4;
int nServiceTimeIdx = 5;
int nPickupIdx = 6;
int nDeliveryIdx = 7;
int nOfType0, nOfType1;
float capOfType0, capOfType1, costOfType0, costOfType1;
float costByCapacityOfType0;
float costByCapacityOfType1;

struct Sol
{
    int NV;
    float TC; // TC = running + fixed cost
    float SD;
    float WT;

    vector<int> vectVehType;
    vector<vector<int>> vectRoute;
    vector<float> vectDuration;

    bool operator<(const Sol &other) const
    {
        if (TC != other.TC)
            return TC < other.TC;
        if (SD != other.SD)
            return SD < other.SD;
        if (WT != other.WT)
            return WT < other.WT;
        return NV < other.NV;
    }
    bool operator==(const Sol &other) const
    {
        return NV == other.NV && TC == other.TC && SD == other.SD && WT == other.WT;
    }
};

float dist(vector<int> vTask1, vector<int> vTask2)
{
    float fVal = sqrt(pow(vTask1.at(0) - vTask2.at(0), 2) + pow(vTask1.at(1) - vTask2.at(1), 2));
    return floor(fVal * 100.0) / 100.0;
}

// updated TC with fixed cost
Sol populateSolution(Sol SolX, unordered_map<int, vector<int>> mapTask)
{
    // cout << "START" << endl;
    for (int i = 0; i < SolX.vectRoute.size();)
    {
        if (SolX.vectRoute[i].size() == 0)
            SolX.vectRoute.erase(SolX.vectRoute.begin() + i);
        else
            ++i;
    }

    int NV = SolX.vectRoute.size();
    float TC = 0, SD = 0, WT = 0;
    vector<float> vectSD(SolX.vectRoute.size(), 0);
    SolX.vectDuration = vector<float>(SolX.vectRoute.size(), 0);

    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        if (SolX.vectVehType.at(i) == 0)
            TC += costOfType0; // here
        else
            TC += costOfType1; // and here

        for (int j = 0; j < SolX.vectRoute.at(i).size(); j++)
        {
            int nCurTask = SolX.vectRoute[i][j];
            if (j == 0)
            {
                TC += dist(mapTask.at(nCurTask), mapTask.at(0));
                vectSD[i] += dist(mapTask.at(nCurTask), mapTask.at(0));
            }
            if (j > 0)
            {
                TC += dist(mapTask.at(SolX.vectRoute[i][j - 1]), mapTask.at(nCurTask));
                vectSD[i] = vectSD[i] + dist(mapTask.at(SolX.vectRoute[i][j - 1]), mapTask.at(nCurTask));
            }
            if (vectSD[i] < mapTask.at(nCurTask).at(nStartTimeIdx))
            {
                WT += mapTask.at(nCurTask).at(nStartTimeIdx) - vectSD[i];
                vectSD[i] = mapTask.at(nCurTask).at(nStartTimeIdx) + mapTask.at(nCurTask).at(nServiceTimeIdx);
            }
            else
            {
                vectSD[i] += mapTask.at(nCurTask).at(nServiceTimeIdx);
            }
            if (j == SolX.vectRoute.at(i).size() - 1)
            {
                TC += dist(mapTask.at(nCurTask), mapTask.at(0));
                vectSD[i] += dist(mapTask.at(nCurTask), mapTask.at(0));
            }
        }
        
        // cout << "SD at " << i << " = " << vectSD[i] << endl;
        
        SD += vectSD[i];
        SolX.vectDuration.at(i) = vectSD[i];
    }
    // cout << "SD = " << SD << endl;

    SolX.TC = TC;
    SolX.SD = SD;
    SolX.NV = NV;
    SolX.WT = WT;

    return SolX;
}

double cost(Sol SolX, int alpha = 100000, int beta = 10000, int gamma = 1000, int delta = 100)
{
    return SolX.NV * alpha /* multiple it fizxed cost */ + SolX.TC * beta + SolX.SD * gamma + SolX.WT * delta;
}

float distCost(unordered_map<int, vector<int>> mapTask, vector<int> vctRoute)
{
    float TC = 0;
    for (int i = 0; i < vctRoute.size(); i++)
    {
        if (i == 0 || i == vctRoute.size() - 1)
        {
            TC += dist(mapTask.at(vctRoute.at(i)), mapTask.at(0));
        }
        else
        {
            TC += dist(mapTask.at(vctRoute.at(i - 1)), mapTask.at(vctRoute.at(i)));
        }
    }
    return TC;
}

bool checkRouteFeasibility(vector<int> vRoute, unordered_map<int, vector<int>> mapTask, int nMaxLoad)
{
    int nLoad = 0;
    float SD = 0;
    for (int i = 0; i < vRoute.size(); i++)
    {
        nLoad += mapTask.at(vRoute[i]).at(nDemandIdx);
        if (nLoad > nMaxLoad)
        {
            return false;
        }
    }
    for (int i = 0; i < vRoute.size(); i++)
    {
        float nStartTime = mapTask.at(vRoute[i]).at(nStartTimeIdx);
        if (i == 0)
        {
            SD = dist(mapTask.at(0), mapTask.at(vRoute[i]));
            // cout << "1. SD = " << SD << endl;
        }
        if (i > 0)
        {
            SD += dist(mapTask.at(vRoute[i - 1]), mapTask.at(vRoute[i]));
            // cout << "2. SD = " << SD << endl;
        }
        
        if (SD < nStartTime) {
            // cout << "nStartTime = " << nStartTime << endl;
            SD = nStartTime + mapTask.at(vRoute[i]).at(nServiceTimeIdx);
            // cout << "3. SD = " << SD << endl;
        }
        else {
            if (SD > mapTask.at(vRoute[i]).at(nEndTimeIdx))
            {
                return false;
            }
            SD += mapTask.at(vRoute[i]).at(nServiceTimeIdx);
            // cout << "4. SD = " << SD << endl;
        }

    }

    SD += dist(mapTask.at(0), mapTask.at(vRoute[vRoute.size() - 1]));
    if (SD > mapTask.at(0).at(nEndTimeIdx))
    {
        return false;
    }

    // cout << "SD = " << SD << endl;
    if (SD > SDLIMIT)
    {
        return false;
    }
    return true;
}

Sol insertPDPair(Sol SolX, int pTask, unordered_map<int, vector<int>> mapTask, int nOfType0, int nOfType1)
{
    double minCost = DBL_MAX;
    int nRouteIdx;
    int nMaxLoad;
    int fixedCost;
    vector<int> vRoute;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        if (SolX.vectVehType.at(i) == 0)
        {
            nMaxLoad = capOfType0;
            fixedCost = costOfType0;
        }
        else
        {
            nMaxLoad = capOfType1;
            fixedCost = costOfType1;
        }

        for (int j = 0; j < SolX.vectRoute.at(i).size() + 1; j++)
        {
            vector<int> tempRoute = SolX.vectRoute[i];
            tempRoute.insert(tempRoute.begin() + j, pTask);
            for (int k = j + 1; k < tempRoute.size() + 1; k++)
            {
                tempRoute.insert(tempRoute.begin() + k, mapTask.at(pTask).at(nDeliveryIdx));
                if (checkRouteFeasibility(tempRoute, mapTask, nMaxLoad))
                {
                    // updated
                    float thisCost = distCost(mapTask, tempRoute) + fixedCost;
                    if (minCost > thisCost)
                    {
                        minCost = thisCost;
                        vRoute = tempRoute;
                        nRouteIdx = i;
 
                    }
                }
                tempRoute.erase(tempRoute.begin() + k);
            }
            tempRoute.erase(tempRoute.begin() + j);
        }
    }
    // cout << pTask << endl;
    if (vRoute.empty())
    {
        int f = 0;
        int defaultVehType;
        // int s = 0;
        if (nOfType1 > 0 || nOfType0 > 0)
        {
            // cout << "1\n";
            if (nOfType0 == 0)
            {
                // A is not having vehicles, B is having vehicles
                if (capOfType1 < mapTask[pTask].at(nDemandIdx))
                    return SolX;
                defaultVehType = 1;
                // nOfType1--;
            }
            else
            {
                // A is having vehicles, B may or may not
                if (nOfType1 > 0)
                {
                    // both A and B are having vechiles
                    if (costByCapacityOfType0 < costByCapacityOfType1)
                    {
                        // s = 1;
                        if (capOfType0 < mapTask[pTask].at(nDemandIdx))
                            return SolX;
                        defaultVehType = 0;
                        // nOfType0--;
                    }
                    else
                    {
                        // s = 2;
                        if (capOfType1 < mapTask[pTask].at(nDemandIdx))
                            return SolX;
                        defaultVehType = 1;
                        // nOfType1--;
                    }
                }
                else
                {
                    // s = 3;
                    // A is having vehicles, B isn't
                    if (capOfType0 < mapTask[pTask].at(nDemandIdx))
                        return SolX;
                    defaultVehType = 0;
                    // nOfType0--;
                }
            }

            vector<int> vNewRoute;
            vNewRoute.push_back(pTask);
            vNewRoute.push_back(mapTask.at(pTask).at(nDeliveryIdx));
            // SolX.vectRoute.push_back(vNewRoute);
            // SolX.vectVehType.push_back(defaultVehType);
                
            if (defaultVehType == 0) {
                nMaxLoad = capOfType0;
            }
            else {
                nMaxLoad = capOfType1;
            }

            if (checkRouteFeasibility(vNewRoute, mapTask, nMaxLoad)) {
                SolX.vectRoute.push_back(vNewRoute);
                SolX.vectVehType.push_back(defaultVehType);
            }
        }
        else {
            return SolX;
        }
    }
    else
    {
        SolX.vectRoute.at(nRouteIdx) = vRoute;
    }
    return SolX;
}

// need to modify
void printSolution(Sol SolX)
{
    std::cout << setw(25) << left << "Objective cost" << setw(1) << left << ":" << cost(SolX) << endl;
    std::cout << setw(25) << left << "Number of vehicles" << setw(1) << left << ":" << SolX.NV << endl;
    std::cout << setw(25) << left << "Total cost" << setw(1) << left << ":" << SolX.TC << endl;
    std::cout << setw(25) << left << "Schedule duration" << setw(1) << left << ":" << SolX.SD << endl;
    std::cout << setw(25) << left << "Average service duration" << setw(1) << left << ":" << SolX.SD / SolX.NV << endl;
    std::cout << setw(25) << left << "Waiting time" << setw(1) << left << ":" << SolX.WT << endl;
    std::cout << setw(5) << left << "#" << setw(5) << left << "Type" << setw(9) << left << "Duration" << setw(40) << left << "Route" << endl;
    for (int i = 0; i < SolX.vectRoute.size(); i++)
    {
        std::cout << setw(5) << left << i + 1 << setw(5) << left << SolX.vectVehType.at(i) << setw(9) << left << SolX.vectDuration.at(i);
        for (int j = 0; j < SolX.vectRoute.at(i).size(); j++)
        {
            std::cout << SolX.vectRoute.at(i).at(j) << " ";
        }
        std::cout << endl;
    }
    return;
}

// need to modify
Sol createInitialSolution(unordered_map<int, vector<int>> mapTask, /*0:Max Comb RTT | 1:Min DL*/ int method, unordered_set<int> setTask)
{
    Sol SolX;
    Sol Solution;
    int nDepot = 0;
    int nInitPickup;
    int nInitDelivery;
    int nEarliestTaskTime = INT_MAX;
    int nEarliestTaskNo = INT_MAX;
    int nOfType0 = ::nOfType0, nOfType1 = ::nOfType1;

    int defaultVehType = 0;

    if (method == 0)
    {
        float fMaxRoundTrip = 0;
        for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
        {
            if (itr->first != 0 && itr->second.at(nPickupIdx) == 0)
            {                                                                                         // select only non depo and pickup task
                float fDepotToP = dist(mapTask.at(nDepot), itr->second);                              // distance: depot to Pickup
                float fPToD = dist(itr->second, mapTask.at(itr->second.at(nDeliveryIdx)));            // distance: pickup to delivery
                float fDToDepot = dist(mapTask.at(itr->second.at(nDeliveryIdx)), mapTask.at(nDepot)); // distance: delivery to depot
                float fCurRoundTrip = fDepotToP + fPToD + fDToDepot;                                  // round trip distance

                if (fCurRoundTrip > fMaxRoundTrip)
                {
                    fMaxRoundTrip = fCurRoundTrip;
                    nInitPickup = itr->first;
                    nInitDelivery = itr->second.at(nDeliveryIdx);
                }
            }
        }

        // Type 1 has lower cost
        if (costByCapacityOfType0 > costByCapacityOfType1)
        {
            // capcity of type 1 is more than demand
            if (capOfType1 >= mapTask[nInitPickup].at(nDemandIdx))
            {
                defaultVehType = 1;
                nOfType1--;
            }
            // capacity of type 1 is less than demand
            else
            {
                // capcity of type 0 is okay
                if (capOfType0 >= mapTask[nInitPickup].at(nDemandIdx))
                {
                    nOfType0--;
                }
                // capacity of both are less
                else
                {
                    cout << "Capacity of both vehicle type is smaller than demand.\n";
                    exit(0);
                }
            }
        }
        // type 0 has lower cost
        else
        {
            // 0 has enough capacity
            if (capOfType0 >= mapTask[nInitPickup].at(nDemandIdx))
            {
                defaultVehType = 0;
                nOfType0--;
            }
            // 0 has no enough capacity
            else
            {
                // capcity of type 1 is more than demand
                if (capOfType1 >= mapTask[nInitPickup].at(nDemandIdx))
                {
                    defaultVehType = 1;
                    nOfType1--;
                }
                // capacity of both are less
                else
                {
                    cout << "Capacity of both vehicle type is smaller than demand.\n";
                    exit(0);
                }
            }
        }

        vector<int> vInitRoute;
        vInitRoute.push_back(nInitPickup);
        vInitRoute.push_back(nInitDelivery);

        SolX.vectRoute.push_back(vInitRoute);
        SolX.vectVehType.push_back(defaultVehType);
        SolX = populateSolution(SolX, mapTask);

        // cout << "Solution after inserting frist PD pair based on max round trip";
        // printSolution(SolX);
        // cout << endl << "nOfType0 left = " << nOfType0 << endl;
        // cout << "nOfType1 left = " << nOfType1 << endl << endl;

        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        // cout << "setTask.size() = " << setTask.size() << endl;

        vector<pair<float, int>> rtDistanceToTask;
        for (auto it = mapTask.begin(); it != mapTask.end(); it++)
        {
            if (setTask.find(it->first) != setTask.end() && it->first != 0 && it->second.at(nPickupIdx) == 0)
            {
                float fDepotToP = dist(mapTask.at(nDepot), it->second);                              // distance: depot to Pickup
                float fPToD = dist(it->second, mapTask.at(it->second.at(nDeliveryIdx)));             // distance: pickup to delivery
                float fDToDepot = dist(mapTask.at(it->second.at(nDeliveryIdx)), mapTask.at(nDepot)); // distance: delivery to depot
                float fCurRoundTrip = fDepotToP + fPToD + fDToDepot;
                rtDistanceToTask.push_back(make_pair(fCurRoundTrip, it->first));
            }
        }

        sort(rtDistanceToTask.begin(), rtDistanceToTask.end());
        std::reverse(rtDistanceToTask.begin(), rtDistanceToTask.end());

        float thisCost;
        int k = 0;
        while (!setTask.empty())
        {
            // cout << endl << "setTask.size() = " << setTask.size() << endl;
            // if (setTask.size() == 14) {
            //     printSolution(SolX);
            // }
            int nSolTaskNo;
            int nIdx = -1;
            float minCost = FLT_MAX;

            // cout << "rtDistanceToTask.size() = " << rtDistanceToTask.size() << endl;
            for (int i = 0; i < rtDistanceToTask.size(); i++)
            {
                // cout << "\ti = " << i << endl;
                if (setTask.find(rtDistanceToTask[i].second) != setTask.end())
                {

                    int nPickupTask = rtDistanceToTask[i].second;
                    // cout << "\tnPickupTask = " << nPickupTask << endl;
                    Sol SolY = insertPDPair(SolX, nPickupTask, mapTask, nOfType0, nOfType1);
                    SolY = populateSolution(SolY, mapTask);
                    // cout << "\tCheck-in\n";
                    if (SolX == SolY)
                    {
                        continue;
                    }
                    // cout << "\tCheck-out\n";
                    thisCost = SolY.TC;
                    if (thisCost < minCost)
                    {
                        minCost = thisCost;
                        nSolTaskNo = nPickupTask;
                        nIdx = i;
                        Solution = SolY;
                    }
                }
            }
            // cout << "for exited\n";

            if (nIdx != -1)
            {
                // cout << "Picked " << nSolTaskNo << endl;
                SolX = Solution;
                nOfType0 = ::nOfType0 - count(SolX.vectVehType.begin(), SolX.vectVehType.end(), 0);
                nOfType1 = ::nOfType1 - count(SolX.vectVehType.begin(), SolX.vectVehType.end(), 1);
                // printSolution(SolX);
                // cout << "nOfType0 left = " << nOfType0 << endl;
                // cout << "nOfType1 left = " << nOfType1 << endl;
                setTask.erase(nSolTaskNo);
                setTask.erase(mapTask.at(nSolTaskNo).at(nDeliveryIdx));
                rtDistanceToTask.erase(rtDistanceToTask.begin() + nIdx);
                continue;
            }
            else if (k < 2)
            {
                // cout << "k = "<< k << endl;
                ++k;
                continue;
            }
            else
            {
                cout << "Increase no of vehicles or schedule duration limit.\n";
                exit(0);
            }
        }
    }
    // haven't updated else part
    else
    {
        for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
        {
            if (itr->first != 0 && itr->second.at(nDeliveryIdx) == 0)
            {
                if (itr->second.at(nEndTimeIdx) < nEarliestTaskTime)
                {
                    nEarliestTaskTime = itr->second.at(nEndTimeIdx);
                    nInitPickup = itr->second.at(nPickupIdx);
                    nInitDelivery = itr->first;
                }
            }
        }

        vector<int> vInitRoute;
        vInitRoute.push_back(nInitPickup);
        vInitRoute.push_back(nInitDelivery);
        SolX.vectRoute.push_back(vInitRoute);
        SolX = populateSolution(SolX, mapTask);
        setTask.erase(nInitPickup);
        setTask.erase(nInitDelivery);

        vector<pair<int, int>> endTimesToTask;
        for (auto it = mapTask.begin(); it != mapTask.end(); it++)
        {
            if (it->first != 0 && it->second.at(nDeliveryIdx) == 0)
            {
                endTimesToTask.push_back(make_pair(it->second.at(nEndTimeIdx), it->first));
            }
        }

        sort(endTimesToTask.begin(), endTimesToTask.end());

        float thisCost;
        while (!setTask.empty())
        {
            int nSolTaskNo;
            int nIdx;
            float minCost = FLT_MAX;
            for (int i = 0; i < endTimesToTask.size(); i++)
            {
                if (setTask.find(endTimesToTask[i].second) != setTask.end())
                {
                    int nPickupTask = mapTask.at(endTimesToTask[i].second).at(nPickupIdx);
                    Sol SolY = insertPDPair(SolX, nPickupTask, mapTask, nOfType0, nOfType1);
                    SolY = populateSolution(SolY, mapTask);
                    thisCost = SolY.TC;
                    if (thisCost < minCost)
                    {
                        minCost = thisCost;
                        nSolTaskNo = nPickupTask;
                        nIdx = i;
                        Solution = SolY;
                    }
                }
            }
            SolX = Solution;
            setTask.erase(nSolTaskNo);
            setTask.erase(mapTask.at(nSolTaskNo).at(nDeliveryIdx));
            endTimesToTask.erase(endTimesToTask.begin() + nIdx);
        }
    }
    // cout << "exiting\n";
    return SolX;
}

void readData(string strFileName, unordered_map<int, vector<int>> &mapTask, unordered_set<int> &setTask)
{

    ifstream MyFile;
    MyFile.open(strFileName);
    string line;
    int i = 0;
    while (MyFile.good())
    {
        vector<int> value;
        getline(MyFile, line);
        istringstream iss(line);
        int j = 0;
        int key;

        do
        {
            int subs;
            iss >> subs;

            if (i == 0)
            {
                if (j == 0)
                    nOfType0 = subs;
                if (j == 1)
                    capOfType0 = subs;
                if (j == 2)
                    costOfType0 = subs;
                if (j == 3)
                    nOfType1 = subs;
                if (j == 4)
                    capOfType1 = subs;
                if (j == 5)
                    costOfType1 = subs;
            }
            else
            {
                if (j == 0)
                    key = subs;
                else
                    value.push_back(subs);
            }
            ++j;
        } while (iss);

        if (i > 0)
        {
            mapTask[key] = value;
        }
        ++i;
    }

    MyFile.close();

    for (auto itr = mapTask.begin(); itr != mapTask.end(); itr++)
    {
        if (itr->first != 0)
        {
            setTask.insert(itr->first);
        }
    }
    if (setTask.size() % 2 == 1)
    {
        std::cout << "Solution requires even number of PD locations\n";
    }

    return;
}

int main()
{
    clock_t tStart = clock();
    unordered_map<int, vector<int>> mapTask;
    unordered_set<int> setTask;

    string path = "D:/Projects/VRPTW/lr101.txt";
    // first row <#Type0, capacityType0, costType0, #Type1, capacityType1, costType1>

    cout << path << endl;
    readData(path, mapTask, setTask);

    costByCapacityOfType0 = costOfType0 / capOfType0;
    costByCapacityOfType1 = costOfType1 / capOfType1;

    // bool a = checkRouteFeasibility({11,1}, mapTask, 100);
    cout << "Schedule duration limit set at: " << SDLIMIT << endl;
    cout << "NV for type 0 set at          : " << nOfType0 << endl;
    cout << "NV for type 1 set at          : " << nOfType1 << endl;
    

    // cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Read the input file.\n";

    Sol S = createInitialSolution(mapTask, 0, setTask);
    // cout << (int)(clock() - tStart) / CLOCKS_PER_SEC << ": Initial solution created.\n";
    cout << "----------------------------------------" << endl;
    printSolution(S);

    return 0;
}