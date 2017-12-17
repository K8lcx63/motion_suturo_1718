//
// Created by menan on 12/17/17.

#ifndef MOTION_KITCHENCOLLISIONOBJECTSERVICE_H
#define MOTION_KITCHENCOLLISIONOBJECTSERVICE_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>

class KitchenCollisionObjectService {
public:
    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res);
};


#endif //MOTION_KITCHENCOLLISIONOBJECTSERVICE_H
