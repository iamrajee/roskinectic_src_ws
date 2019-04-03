#include<stdio.h>
#include<time.h>
#include<stdlib.h>

void main()
{
    int tortoise=0;
    int hare=0;
    int random;
    int finish=70;
 
 
    printf("The Race Between The Hare and The Tortoise/n");
    printf("Bang!! And they’re off!!");
 
    srand(time(NULL));
 
    random= rand()%10 +1;
 
    for(int race=1; tortoise !=finish && hare!=finish; race++)
    {
        //for tortoise
        if (random>=1 && random<=5)
        {
            tortoise+=3;
        }
        else if (random>=6 && random<=7)
        {
            tortoise-=6;
        }
        else if(random>=8 && random<=10)
        {
            tortoise+=1;
        }
        else
        {
            printf(" ", race);
        }
        printf("T", tortoise);
        //for hare
        if (random>=1 && random<=2)
        {
            hare=hare;
        }
        else if (random>=3 && random<=4)
        {
            hare+=9;
        }
        else if(random==5)
        {
            hare-=12;
        }
        else if(random>=6 && random<=8)
        {
            hare+=1;
        }
        else if(random>=9 && random<=10)
        {
            hare-=2;
        }
        {
            printf(" ", race);
        }
        printf("H", hare);
 
    }
 
}