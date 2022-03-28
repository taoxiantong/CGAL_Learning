// make in 2021.12.05

#include "shape_detect.h"

int main(int argc, char *argv[]) {
    std::cout << std::endl <<
              "region_growing_on_polygon_mesh example started"
              << std::endl << std::endl;
    // Load off data either from a local folder or a user-provided file.
    std::ifstream in(argc > 1 ? argv[1] : "../../data/polygon_mesh.off");
    CGAL::IO::set_ascii_mode(in);
    if (!in) {
        std::cout << "Error: cannot read the file polygon_mesh.off!" << std::endl;
        std::cout << "You can either create a symlink to the data folder or provide this file by hand." << std::endl << std::endl;
        return EXIT_FAILURE;
    }
    SMesh polygon_mesh;

    in >> polygon_mesh;
    in.close();

    const Face_range face_range = faces(polygon_mesh);
    std::cout <<"* polygon mesh with "<< face_range.size() << " faces is loaded" << std::endl;
    // Default parameter values for the data file polygon_mesh.off.
    // Eplison 1.91816
    // Normal Tol in degrees 30
    // Minimum Number of Items 322
    // Search Rigorosity  0.05
    // connectiviy Epsilon 0.002
    const FT          max_distance_to_plane = FT(1.91816);      // 0.01 * 距离
    const FT          max_accepted_angle    = FT(30);
    const std::size_t min_region_size       = 322;              // 0.01 * 所有面数
    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(polygon_mesh);
    const Vertex_to_point_map vertex_to_point_map(get(CGAL::vertex_point, polygon_mesh));
    Region_type region_type(polygon_mesh,max_distance_to_plane, max_accepted_angle, min_region_size, vertex_to_point_map);
    // Sort face indices.
    Sorting sorting(polygon_mesh, neighbor_query, vertex_to_point_map);
    sorting.sort();
    // Create an instance of the region growing class.
    Region_growing region_growing( face_range, neighbor_query, region_type, sorting.seed_map());
    // Run the algorithm.
    Regions regions;
    region_growing.detect(std::back_inserter(regions));
    // Print the number of found regions.
    std::cout << "* " << regions.size() << " regions have been found" << std::endl;


    // 定义另一个网格，并用颜色显示
    SMesh FaceMesh;
    FaceMesh = polygon_mesh;


    // 定义表面颜色
    const Face_range fr = faces(FaceMesh);
    typename SMesh::Property_map<Face_index, Color> face_color;

    // 帮绑定颜色和网格
    bool created;
    boost::tie(face_color, created) = FaceMesh.add_property_map<Face_index, Color>("f:color", Color(0, 0, 0));
    if (!created) {
        std::cout << std::endl <<"region_growing_on_polygon_mesh example finished  Face Color Failed" << std::endl << std::endl;
        return EXIT_FAILURE;
    }

    // 保存位置
    const std::string path     = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/";
    const std::string fullpath = path + "regions_polygon_mesh.off";
    const std::string savepath = path + "regions_polygon_mesh_save.off";
    std::ofstream out(fullpath);

    // 保存
    out << FaceMesh;
    out.close();

    //==================================================
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
    // Fout<<str_line;  Fout<<" ";
    // 写入空格，后续直接写入RGB 和回车
    //==================================================

    
    // 迭代所有表面
    srand(static_cast<unsigned int>(time(nullptr)));
    int i = 0;
    for (const auto& region : regions) {
        // 生成随机颜色
        CGAL::Random rnd(static_cast<unsigned int>(i));
        const Color color(
                static_cast<unsigned char>( rnd.get_int(0, 255)),
                static_cast<unsigned char>(rnd.get_int(0, 255)),
                static_cast<unsigned char>(  rnd.get_int(0, 255)));
        // Iterate through all region items.
        using size_type = typename SMesh::size_type;
        // std::cout<<i++<<std::endl;
        for (const auto index : region) {
            face_color[Face_index(static_cast<size_type>(index))] = color;
        }
        i++;
    }

//    using size_type = typename SMesh::size_type;
//    cout<<face_color[Face_index(static_cast<size_type>(1))];

    for (auto C : face_color ) {
        unsigned int R = (unsigned int)C.r();
        unsigned int G = (unsigned int)C.g();
        unsigned int B = (unsigned int)C.b();
        string rgb = " " + to_string(R) + " " + to_string(G)+ " " + to_string(B) + "\n";
        string meshRGB = str_line + rgb;
        Fout<<meshRGB;
        getline(Fin, str_line);
    }
    Fout.close();

    std::cout << "* polygon mesh is saved in " << fullpath << std::endl;
    // 显示表面检测后的效果

    return EXIT_SUCCESS;
}
