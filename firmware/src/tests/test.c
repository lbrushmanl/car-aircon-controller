#include <stdbool.h>
#include <stdio.h>


int main(void)
{
    int m = 2;
    switch (m)
    {
    case 2:
        if (true)
        {
            printf("supg");
            break;
        }

        printf("oof");
        break;

    case 1:
        printf("cring");
        break;

    default:
        break;
    }

    return 0;
}