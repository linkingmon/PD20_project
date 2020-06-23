#include <stdio.h>
#include <stdlib.h>
#include "flute.h"

extern void readLUT();

int main()
{
    int d=0;
    // int x[MAXD], y[MAXD];
    double x[MAXD], y[MAXD];
    
    Tree flutetree;
    int flutewl;
    
    printf("flute net");
    while (!feof(stdin)) {
        scanf("%lf %lf\n", &x[d], &y[d]);
        d++;
    }
    printf("flute net");
    readLUT();

    flutetree = flute(d, x, y, ACCURACY);
    printf("FLUTE wirelength = %lf\n", flutetree.length);
    printtree(flutetree);

    flutewl = flute_wl(d, x, y, ACCURACY);
    printf("FLUTE wirelength (without RSMT construction) = %lf\n", flutewl);
}
