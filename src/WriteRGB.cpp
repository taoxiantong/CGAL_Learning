//
// Created by txt on 2021/12/9.
//

// make in 2021.12.05

#include "iostream"
#include "fstream"
#include "string"
#include "vector"
using namespace std;

int main(int argc, char *argv[]) {
    // 保存位置
    const std::string path     = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/";
    const std::string fullpath = path + "cube1.off";
    const std::string savepath = path + "cubesave.off";
    std::fstream Fin;
    Fin.open(fullpath, ios::in|ios::out);
    std::fstream Fout;
    Fout.open(savepath);

    string str_line;
    string space_delimiter = " ";
    vector<string> words{};
    int point_num =0;
    while (true)
    {
        size_t pos = 0;
        getline(Fin, str_line);
        Fout<<str_line; Fout<<"\n";
        while ((pos = str_line.find(space_delimiter)) != string::npos) {
            words.push_back(str_line.substr(0, pos));
            str_line.erase(0, pos + space_delimiter.length());
        }
        // 最后一个空格到末尾添加上
        words.push_back(str_line.substr(0, str_line.find(space_delimiter)));
        if (words.size()>2){
            int first = atoi(words[0].c_str());
            if (first > 3)
            {
                point_num = first;
                break;
            }
        }
        cout<<"world: "<< str_line <<endl;
        words.clear();
    }
    cout<<"point size is: "<<point_num<<endl;
    str_line.clear();
    for (int i = point_num-2; i >0; i--) {
        getline(Fin, str_line);
        Fout<<str_line; Fout<<"\n";
    }

    while (getline(Fin, str_line)) {
        size_t pos = 0;
        pos = str_line.find(space_delimiter);
        int first = atoi((str_line.substr(0, pos)).c_str());
        if (first == 3)
            break;
        Fout<<str_line; Fout<<"\n";
    }


    Fout<<str_line;
    string strin(" input\n");
    Fout<<strin;

    getline(Fin, str_line);
    Fout<<str_line;

    Fin.close();
    Fout.close();

    return EXIT_SUCCESS;
}
