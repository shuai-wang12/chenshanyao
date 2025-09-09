#pragma once
#include <vector>
#include <functional>

typedef struct XET
{
    float x;
    float dx,ymax;
    XET* next;
}XET;

template<typename T>
void filling(T& list,std::function<void(int,int)> callback){
    XET *pAET = new XET;
    XET *pNET[1024];
    pAET->next = NULL;

    size_t vertexNum = list.size();

    int maxY = list.back().y;
    int minY = list.back().y;

    //计算最高点y的坐标，扫描线扫到y的最高点结束
    for (size_t i = 1; i < vertexNum; i++)
    {
        if (list[i].y > maxY)
        {
            maxY = list[i].y;
        }

        if (list[i].y < minY)
        {
            minY = list[i].y;
        }
    }

    //初始化NET表，这也是一个有头结点的链表，头结点的dx，x，ymax都初始化为0
    for (int i = 0; i <= maxY; i++)
    {
        pNET[i] = new XET;
        pNET[i]->dx = 0;
        pNET[i]->x = 0;
        pNET[i]->ymax = 0;
        pNET[i]->next = NULL;
    }

    //建立并扫描NET表
    for (int i = minY; i <= maxY; i++)
    {
        /*i表示扫描线，扫描线从多边形的最底端开始，向上扫描*/
        for (size_t j = 0; j < vertexNum; j++)
        {
            /*如果多边形的该顶点与扫描线相交，判断该点为顶点的两条直线是否在扫描线上方
                *如果在上方，就记录在边表中，并且是头插法记录，结点并没有按照x值进行排序，毕竟在更新AET的时候还要重新排一次
                *所以NET表可以暂时不排序
                */
            if (list[j].y == i)
            {
                if (list[(j - 1 + vertexNum) % vertexNum].y > list[j].y)
                {
                    XET *p = new XET;
                    p->x = float(list[j].x);
                    p->ymax = float(list[(j - 1 + vertexNum) % vertexNum].y);
                    p->dx = float((list[(j - 1 + vertexNum) % vertexNum].x - list[j].x)) / float((list[(j - 1 + vertexNum) % vertexNum].y - list[j].y));
                    p->next = pNET[i]->next;
                    pNET[i]->next = p;
                }
                /*笔画后面的那个点*/
                if (list[(j + 1 + vertexNum) % vertexNum].y > list[j].y)
                {
                    XET *p = new XET;
                    p->x = float(list[j].x);
                    p->ymax = float(list[(j + 1 + vertexNum) % vertexNum].y);
                    p->dx = float((list[(j + 1 + vertexNum) % vertexNum].x - list[j].x)) / float((list[(j + 1 + vertexNum) % vertexNum].y - list[j].y));
                    p->next = pNET[i]->next;
                    pNET[i]->next = p;
                }
            }
        }
    }

    /*建立并更新活性边表AET*/
    for (int i = minY; i <= maxY; i++)
    {
        /*更新活性边表AET，计算扫描线与边的新的交点x，此时y值没有达到临界值的话*/
        XET *p = pAET->next;
        while (p)
        {
            p->x = p->x + p->dx;
            p = p->next;
        }

        /*更新完以后，对活性边表AET按照x值从小到大排序*/
        XET *tq = pAET;
        p = pAET->next;
        tq->next = NULL;
        while (p)
        {
            while (tq->next && p->x >= tq->next->x)
                tq = tq->next;
            XET *s = p->next;
            p->next = tq->next;
            tq->next = p;
            p = s;
            tq = pAET;
        }

        /*从AET表中删除ymax==i的结点*/
        XET *q = pAET;
        p = q->next;
        while (p)
        {
            if ((p->ymax - i) < 1e-20)
            {
                q->next = p->next;
                delete p;
                p = q->next;
            }
            else
            {
                q = q->next;
                p = q->next;
            }
        }
        /*将NET中的新点加入AET，并用插入法按X值递增排序*/
        p = pNET[i]->next;
        q = pAET;
        while (p)
        {
            while (q->next && p->x >= q->next->x)
                q = q->next;
            XET *s = p->next;
            p->next = q->next;
            q->next = p;
            p = s;
            q = pAET;
        }

        /*配对填充颜色*/
        p = pAET->next;
        while (p && p->next)
        {
            for (float j = p->x; j <= p->next->x; j++)
            {
                callback((int)j,i);
            }
            p = p->next->next;
        }
    }
}