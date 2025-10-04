#include <iostream>
using namespace std;

class Material
{
public:
    Material();
    ~Material();
    void print();

};

// 实现构造函数
Material::Material()
{
    cout << "Material构造函数被调用" << endl;
}

// 实现析构函数
Material::~Material()
{
    cout << "Material析构函数被调用" << endl;
}

// 实现print函数
void Material::print()
{
    cout << "Material print函数被调用" << endl;
}

int main()
{
    cout << "--- 构造函数 ---" << endl;
    Material m;

    cout << "\n--- print函数 ---" << endl;
    m.print();

    cout << "\n--- 自动进行析构函数 ---" << endl;
    return 0;
}