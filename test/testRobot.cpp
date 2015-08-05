#include <gtest/gtest.h>

//function for example
int add(int a, int b) {
    return a+b;
}

TEST(Example,testcase1)
{
    EXPECT_EQ(5,add(2,3));
}

int main(int argc,char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
