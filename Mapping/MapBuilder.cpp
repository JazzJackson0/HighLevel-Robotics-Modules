#include "MapBuilder.hpp"



MapBuilder::MapBuilder() { /*Default Constructor*/ }

MapBuilder::MapBuilder(int m, int n) : M(m), N(n) {
    P = -1;
}

MapBuilder::MapBuilder(int p, int m, int n) : P(p), M(m), N(n) {}


void MapBuilder::Update_2DMapDimensions(int m, int n) {

    M = m;
    N = n;
}


void MapBuilder::Update_3DMapDimensions(int m, int n, int p) {

    M = m;
    N = n;
    P = p;
}



void MapBuilder::Tensor2D_to_MapFile(Tensor<float, 2> tensor, std::string filename, int filetype, int max_value) {

    map_file.open(filename, std::ios::out | std::ios::trunc);
    
    if (map_file.is_open()) {

        if (filetype == PBM) { map_file << "P4" << std::endl; }

        else if (filetype == PGM) { map_file << "P5" << std::endl; }

        else if (filetype == PPM) { map_file << "P6" << std::endl; }

        // Set Dimensions
        map_file << std::to_string(M) << " " << std::to_string(N) << std::endl;

        // Set Max Value
        if (filetype == PGM || filetype == PPM) {

            map_file << std::to_string(max_value) << std::endl;
        }

        // Populate Raster
        for (int i = 0; i < M; i++) {

            for (int j = 0; j < N; j++) {

                if (tensor(i, j) > 0.1) { map_file << "1 "; }
                else { map_file << "0 "; }        
            }
            map_file << std::endl;
        }


        map_file.close();
    }
    
}


Tensor<float, 2> MapBuilder::MapFile_to_Tensor2D(std::string filename, int filetype) {

    Tensor<float, 2> tensor_2d(M, N);
    tensor_2d.setZero();
    std::vector<std::string> elems;

    map_file.open(filename, std::ios::in);

    if (map_file.is_open() && (filetype == PBM || filetype == PGM || filetype == PPM)) { 
        std::string line;

        int line_num = 0;
        int m = 0;
        while (getline(map_file, line)) {
            
            line_num++;
            elems = split(line, "\\s+");

            // Skip blank lines
            if (elems.size() == 0)
                continue;
            
            // Skip Comments
            if (elems[0].compare("#") == 0) {
                line_num--;
                continue;
            }

            // Verify Width & Height
            if (line_num == 2 && elems.size() == 2) {
                if (std::stoi(elems[0]) != N || std::stoi(elems[1]) != M) {
                    std::cout << "Map File and Tensor Dimensions Do Not Match. [Map File--> Width: " << elems[0] 
                        << ", Height: " << elems[1] << "] [Tensor--> Width: " << N << ", " << M << "]" << std::endl;
                    break;
                }
            }

            // Enter Raster Section (Version 1)
            else if (filetype == PBM && line_num >= 3) {

                // Populate Tensor
                for (int n = 0; n < elems.size(); n++) {

                    tensor_2d(m, n) = std::stof(elems[n]);
                }
                m++;
            }

            // Enter Raster Section (Version 2)
            else if ((filetype == PGM || filetype == PPM) && line_num >= 3) {

                // Populate Tensor
                for (int n = 0; n < elems.size(); n++) {

                    tensor_2d(m, n) = std::stof(elems[n]);
                }
                m++;
            } 
        }

        map_file.close();

    }
    else  { std::cout << "Error opening file" << std::endl; }

    // Test
    // std::cout << tensor_2d << std::endl;

    return tensor_2d;
}


void MapBuilder::Tensor3D_to_MapFile(Tensor<float, 3> tensor, std::string filename, int filetype) {

    if (P <= 0) { return; }

    map_file.open(filename, std::ios::out | std::ios::trunc);
   
    if (map_file.is_open() && filetype == PLY) {

        std::string header = "ply\nformat ascii 1.0\nelement vertex ";
        header.append(std::to_string(N*M*P));
        header.append("\nproperty float x\nproperty float y\nproperty float z\nend_header");

        map_file << header << std::endl;

        // Add Vertices to Map File
        for (int n = 0; n < N; n++) {

            for (int m = 0; m < M; m++) {

                for (int p = 0; p < P; p++) {

                    map_file << std::to_string(n) << " " 
                        << std::to_string(m) << " " << std::to_string(p) << std::endl;
                }
            }
        }

        map_file.close();
    }
}


Tensor<float, 3> MapBuilder::MapFile_to_Tensor3D(std::string filename, int filetype) {

    // Use PLY Library (7-Libraries/PLY-Polygon-File-Format/)
    Tensor<float, 3> tensor_3d(P, M, N);
    tensor_3d.setZero();
    if (P <= 0) { return tensor_3d; }

    std::vector<std::string> elems;
    map_file.open(filename, std::ios::in);

    if (map_file.is_open() && filetype == PLY) { 
        std::string line;

        bool in_header = true;
        int points_added = 0;
        while (getline(map_file, line)) {
            
            elems = split(line, "\\s+");
            
            // Skip blank lines
            if (elems.size() == 0)
                continue;

            if (in_header && elems[0].compare("end_header") == 0) {

                in_header = false;
                continue;
            }

            // Verify Map Data fits Tensor Dimensions
            if (in_header && elems.size() == 3) {

                if (elems[1].compare("vertex") == 0 && std::stoi(elems[2]) != (N*M*P)) {
                    std::cout << "Map File and Tensor Dimensions Do Not Match" << std::endl;
                }
            }

            // Not in Header and still have points to add to tensor
            else if (!in_header && points_added < (N*M*P)) {

                // Populate Tensor
                tensor_3d(std::stoi(elems[2]), std::stoi(elems[1]), std::stoi(elems[0])) = 1.f;
            }
        }
        map_file.close();
    }
    else  { std::cout << "Error opening file" << std::endl; }

    // Test
    // std::cout << tensor_3d << std::endl;

    return tensor_3d;
}


VectorXi MapBuilder::MapCoordinate_to_DataStructureIndex(VectorXf coordinate) {

    VectorXi index = VectorXi::Zero(2);
    if (P <= 0) {index.resize(3);}

    index[0] = (int) round(coordinate[0] + (N / 2));
    index[1] = (int) round(coordinate[1] + (M / 2));

    if (coordinate.rows() == 3) {

        index[2] = (int) round(coordinate[2] + (P / 2));
    }
    
    return index;
}


VectorXf MapBuilder::DataStructureIndex_to_MapCoordinate(VectorXi index) {

    VectorXf coordinate = VectorXf::Zero(2);
    if (P <= 0) {coordinate.resize(3);}

    coordinate[0] =  (float) (index[0] + (N / 2));
    coordinate[1] = (float) (index[1] + (M / 2));

    if (index.rows() == 3) {

        coordinate[2] = (float) (index[2] + (P / 2));
    }

    return coordinate;
}