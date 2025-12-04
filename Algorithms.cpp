#include <string>
#include <vector>
#include <iostream>
//#include <bits/stdc++.h>

struct Vec2{
    int x,y;

    Vec2 operator-(const Vec2& other){
        return {x - other.x, y - other.y};
    }
};


class Map{
    private:
        int column;
        int row;
        Vec2 origin;
        double cell_size;

        std::vector<int> map;
    
    public:
        Map(int r,int c,Vec2 o,double size){
            column = c;
            row = r;
            origin = o;
            cell_size = size;

            map.resize(r * c, 0);
        }

        int index(int x, int y){
            return (y * column + x);
        }
        
        void update_cell(bool north, bool east, bool south, bool west, int coord_x, int coord_y){
            int i = index(coord_x, coord_y);
            int cell_num = 1 * north + 2 * east + 4 * south + 8 * west;
            map[i] |= cell_num; 
        }

        void printMap(){
            for (int i=0; i < row ;i++){
                for (int j=0; j < column ; j++){
                    int k = index(j,i);
                    std::cout << map[k] << ' ' ;
                }
                std::cout << std::endl;
            }
        }

        int getDim(){
            return row, column;
        }

        int getValue(int x, int y){
            int ind = index(x,y);
            return map[ind];
        }
};

// A* needs

struct cell{
    int parent_i, parent_j;
    double f, g, h;
};

bool isValid(int row, int col, Map map){
    int ROW, COLUMN = map.getDim();
    return (row >= 0) && (row < ROW) && (col >=0) && (col < COLUMN);
}

bool isUnblocked(Map map,Vec2 dest, Vec2 curr){
    Vec2 direction = dest - curr;
    const int rowBit[] = {8,0,2};
    const int colBit[] = {1,0,4};

    int checkBit = rowBit[direction.x + 1] + rowBit[direction.y + 1];
    int mapBit = map.getValue(curr.x,curr.y);

    if (checkBit & mapBit){
        int outBit = mapBit ^ checkBit;
    }
}



int main(){
    Vec2 origin = {0,0};
    Map map(3, 3, origin, 0.2);
    map.update_cell(true,true,false,false,1,2);
    map.printMap();
    return 0;
}
