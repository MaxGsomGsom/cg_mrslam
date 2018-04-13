#include "g2o/stuff/command_args.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

using namespace std;
using namespace g2o;


int get_int(string &str, int start, int *end = nullptr) {
    int _end = str.find(" ", start);
    string sub = str.substr(start, _end-start);
    int result = atoi(sub.c_str());
    if (end) *end = _end;
    return result;
}


int main(int argc, char** argv)
{
    string file1, file2, outp;
    CommandArgs arg;
    arg.param("f1", file1, "", "g2o graph 1");
    arg.param("f2", file2, "", "g2o graph 2");
    arg.param("o", outp, "merged.g2o", "output file");
    arg.parseArgs(argc, argv);

    if (file1 == "" || file2 == "") {
        arg.printHelp(cout);
        return 0;
    }

    cout << "Processing..." << endl;

    ifstream stream1, stream2;
    vector<string> vert1, vert2, edg1, edg2, fix1, fix2;
    string str;

    stream1.open(file1);
    while (getline(stream1, str))
    {
        if (str.find("VERTEX_SE2") != string::npos || str.find("ROBOTLASER1") != string::npos)
            vert1.push_back(str);
        else if (str.find("EDGE_SE2") != string::npos)
            edg1.push_back(str);
        else if (str.find("FIX") != string::npos)
            fix1.push_back(str);
    }
    stream1.close();

    stream2.open(file2);
    while (getline(stream2, str))
    {
        if (str.find("VERTEX_SE2") != string::npos || str.find("ROBOTLASER1") != string::npos)
            vert2.push_back(str);
        else if (str.find("EDGE_SE2") != string::npos)
            edg2.push_back(str);
        else if (str.find("FIX") != string::npos)
            fix2.push_back(str);
    }
    stream2.close();

    for (auto it1 = vert1.begin(); it1 != vert1.end(); it1++) {
        if ((*it1).find("ROBOTLASER1") != string::npos) continue;
        for (auto it2 = vert2.begin(); it2 != vert2.end(); it2++) {
            if ((*it2).find("ROBOTLASER1") != string::npos) continue;
            if (get_int(*it1, 11) == get_int(*it2, 11)) {
                vert2.erase(it2);
                it2++;
                vert2.erase(it2);
            }
        }
    }

    int end1, end2;
    for (auto it1 = edg1.begin(); it1 != edg1.end(); it1++) {
        for (auto it2 = edg2.begin(); it2 != edg2.end(); it2++) {
            if (get_int(*it1, 9, &end1) == get_int(*it2, 9, &end2)
                    && get_int(*it1, end1++) == get_int(*it2, end2++))
                edg2.erase(it2);
        }
    }

    for (auto it1 = fix1.begin(); it1 != fix1.end(); it1++) {
        for (auto it2 = fix2.begin(); it2 != fix2.end(); it2++) {
            if (get_int(*it1, 4) == get_int(*it2, 4))
                fix2.erase(it2);
        }
    }


    ofstream out_stream;
    out_stream.open(outp);

    for (auto str3: vert1) out_stream << str3 << endl;
    for (auto str3: vert2) out_stream << str3 << endl;
    for (auto str3: edg1) out_stream << str3 << endl;
    for (auto str3: edg2) out_stream << str3 << endl;

    out_stream.close();

    cout << "Done." << endl;

    return 0;
}
