#include <algorithm>
#include <cassert>
#include "BVH.hpp"

const std::string bvh_method = "SAH"; // BVH or SAH
const size_t SAH_N = 6;
const float inter_cost = 1;
const float traversal_cost = 0.2;

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    if (bvh_method == "BVH") {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    } else {
        // dim：当前bound跨度最长的一个维度
        int max_dim = bounds.maxExtent();
        float interval = bounds.Diagonal()[max_dim] / SAH_N;
        std::vector<Object*> final_left_obj,final_right_obj;

        std::vector<float> centroid_list;
        for (int i = 0; i < objects.size(); ++i)
            centroid_list.push_back(objects[i]->getBounds().Centroid()[max_dim]);
        // check if the centroids are very close, if its closer than "interval" ,the recurse will be endless
        // TODO: if very close, it's unnessesary to slpit, but here still split (to be compatible with other code)
        if (*std::max_element(centroid_list.begin(), centroid_list.end()) -
            *std::min_element(centroid_list.begin(), centroid_list.end()) < interval) {
            final_left_obj = {objects.begin(), objects.begin()+1};
            final_right_obj = {objects.begin()+1, objects.end()};
        } else {
            float min_cost = std::numeric_limits<float>::infinity();
            float total_area = bounds.SurfaceArea();
            for (int i =1; i<SAH_N; i++) {
                std::vector<Object*> left_obj,right_obj;
                Bounds3 left_bound,right_bound;
                for (auto obj: objects) {
                    if (obj->getBounds().Centroid()[max_dim] < bounds.pMin[max_dim] + i*interval) {
                        left_obj.push_back(obj);
                        left_bound = Union(left_bound, obj->getBounds());
                    } else {
                        right_obj.push_back(obj);
                        right_bound = Union(right_bound, obj->getBounds());
                    }
                }
                // need to check if size==0,if so the surfaceArea will be undefined;
                float left_cost = left_obj.size()>0 ? left_bound.SurfaceArea()/total_area*left_obj.size()*inter_cost : 0;
                float right_cost = right_obj.size()>0 ? right_bound.SurfaceArea()/total_area*right_obj.size()*inter_cost : 0;
                float cur_cost = left_cost + right_cost + traversal_cost;
                if (cur_cost!=cur_cost){
                    std::cout << left_bound.SurfaceArea()<<" "<<bounds.SurfaceArea()<<" "<<right_bound.SurfaceArea()
                                <<" "<<left_obj.size()<<" "<<right_obj.size()<<"\n";
                    exit(0);
                }
                if (cur_cost < min_cost) {
                    final_left_obj = std::move(left_obj);
                    final_right_obj =  std::move(right_obj);
                    min_cost = cur_cost;
                }
            }
        }
        
        if (final_left_obj.size()>0)
            node->left = recursiveBuild(final_left_obj);
        if (final_right_obj.size()>0)
            node->right = recursiveBuild(final_right_obj);
        Bounds3 empty;
        node->bounds = Union(node->left ? node->left->bounds :empty,
                             node->right ? node->right->bounds : empty);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    Vector3f invDir(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);

    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = ray.direction.x > 0 ? 0 : 1;
    dirIsNeg[1] = ray.direction.y > 0 ? 0 : 1;
    dirIsNeg[2] = ray.direction.z > 0 ? 0 : 1;

    //如果光线没有与碰撞盒相交，直接返回一个空的
    if (!node->bounds.IntersectP(ray, invDir, dirIsNeg))
    {
        return {};
    }

    //如果碰撞盒不再继续细分，测试碰撞盒内的所有物体是否与光线相交，返回最早相交的
    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);
    }
    
    //测试细分的碰撞盒
    Intersection leaf1 = BVHAccel::getIntersection(node->left, ray);
    Intersection leaf2 = BVHAccel::getIntersection(node->right, ray);
    return leaf1.distance < leaf2.distance ? leaf1 : leaf2;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}