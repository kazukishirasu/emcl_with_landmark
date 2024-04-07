#include <iostream>
#include <list>
#include <vector>
#include <string>

int main() {
    // std::pairを使って、stringとvector<int>のペアを持つlistを作成
    std::list<std::pair<std::string, std::vector<int>>> myList;

    // 要素を追加
    myList.push_back(std::make_pair("First", std::vector<int>{1, 2, 3}));
    myList.push_back(std::make_pair("Second", std::vector<int>{4, 5, 6}));
    myList.push_back(std::make_pair("Third", std::vector<int>{7, 8, 9}));

    // 要素を表示
    for (const auto& pair : myList) {
        std::cout << pair.first << ": ";
        for (int num : pair.second) {
            std::cout << num << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
