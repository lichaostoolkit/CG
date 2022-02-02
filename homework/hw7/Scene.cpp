//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
// 	Intersection inter = intersect(ray);

// 	if (inter.happened)
// 	{
// 		// 如果射线第一次打到光源，直接返回
// 		if (inter.m->hasEmission())
// 		{
// 			if (depth == 0) 
// 			{
// 				return inter.m->getEmission();
// 			}
// 			else return Vector3f(0, 0, 0);
// 		}

// 		Vector3f L_dir(0, 0, 0);
// 		Vector3f L_indir(0, 0, 0);

// 		// 随机 sample 灯光，用该 sample 的结果判断射线是否击中光源
// 		Intersection lightInter;
// 		float pdf_light = 0.0f;
// 		sampleLight(lightInter, pdf_light);

// 		// 物体表面法线
// 		auto& N = inter.normal;
// 		// 灯光表面法线
// 		auto& NN = lightInter.normal;

// 		auto& objPos = inter.coords;
// 		auto& lightPos = lightInter.coords;

// 		auto diff = lightPos - objPos;
// 		auto lightDir = diff.normalized();
// 		float lightDistance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

// 		Ray light(objPos, lightDir);
// 		Intersection light2obj = intersect(light);

// 		// 如果反射击中光源
// 		if (light2obj.happened && (light2obj.coords - lightPos).norm() < 1e-2)
// 		{
// 			Vector3f f_r = inter.m->eval(ray.direction, lightDir, N);
// 			L_dir = lightInter.emit * f_r * dotProduct(lightDir, N) * dotProduct(-lightDir, NN) / lightDistance / pdf_light;
// 		}

// 		if (get_random_float() < RussianRoulette)
// 		{
// 			Vector3f nextDir = inter.m->sample(ray.direction, N).normalized();

// 			Ray nextRay(objPos, nextDir);
// 			Intersection nextInter = intersect(nextRay);
// 			if (nextInter.happened && !nextInter.m->hasEmission())
// 			{
// 				float pdf = inter.m->pdf(ray.direction, nextDir, N);
// 				Vector3f f_r = inter.m->eval(ray.direction, nextDir, N);
// 				L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextDir, N) / pdf / RussianRoulette;
// 			}
// 		}

// 		return L_dir + L_indir;
// 	}

// 	return Vector3f(0, 0, 0);
// }


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // shade(p, wo)
    // 先拿到ray的hitpoint，再用path tracing去渲染hitpoint
    Vector3f hitColor;
    Intersection interP = Scene::intersect(ray);
    if(interP.happened) {
        if (interP.m->hasEmission())
            return interP.m->getEmission();
        Material *P_m = interP.m;
    //    float tnear = kInfinity;
        Vector3f P = interP.coords;
        Vector3f N = interP.normal; // normal
        
        // part1: direct light
        Intersection hit_light_inter;
        float pdf_light;
        sampleLight(hit_light_inter, pdf_light);
        // Get x, ws, NN, emit from inter
        auto lightPos = hit_light_inter.coords;
        auto P2light = lightPos - P;  // light2hitpoint
        auto light_distance = P2light.norm()*P2light.norm();
        auto P2light_dir = P2light.normalized();
        
        // Shoot a ray from p to x
        Ray p_2_x_ray(P, P2light_dir);
        Intersection block_intersection = Scene::intersect(p_2_x_ray);
        Vector3f L_dir;
        Vector3f collisionlight = hit_light_inter.coords - interP.coords;
        // If the ray is not blocked in the middle
        // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
        // 这里不用判断相等，浮点数很难相等，用一个epsilon判断
        // 并且 这个边界值判断对画质影响很大，设的太小，会出现很多黑点
        float epsilon = 1e-2; 
        if ((block_intersection.coords - lightPos).norm() < epsilon) {
            // 注意 wi、wo的方向，eval方法要求的wo是往外方向
            auto f_r = P_m->eval(ray.direction, P2light_dir, N);
            auto emit = hit_light_inter.emit;
            auto NN = hit_light_inter.normal;
            L_dir = emit * f_r * dotProduct(P2light_dir, N) * dotProduct(-P2light_dir, NN) / light_distance / pdf_light;
        }
        // part2: indirect light
        Vector3f L_indir = 0.0;
        // Test Russian Roulette with probability RussianRoulette wi = sample(wo, N)
        if (get_random_float() < RussianRoulette) {
            // Trace a ray r(p, wi)
            // If ray r hit a non-emitting object at q
            // L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
            auto indir_wi = P_m->sample(ray.direction, N);
            Ray indir_ray(P, indir_wi);
            Intersection indir_inter = Scene::intersect(indir_ray);
            if (indir_inter.happened && !indir_inter.m->hasEmission()) {
                auto pdf = interP.m->pdf(ray.direction, indir_wi, N);
                auto f_r = interP.m->eval(ray.direction, indir_wi, N);
                L_indir = castRay(indir_ray, depth+1) * f_r * dotProduct(indir_wi, N) / pdf / RussianRoulette;
            }
        }
        hitColor = L_dir + L_indir;
    }
    return hitColor;
}