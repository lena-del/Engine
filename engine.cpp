#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <functional>
#include <limits>
#include <unordered_map>
#include <stack>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

// ==================== OPTIMIZED STRUCTURES ====================
struct Color {
    uint8_t r, g, b, a;
    Color(uint8_t r = 255, uint8_t g = 255, uint8_t b = 255, uint8_t a = 255) 
        : r(r), g(g), b(b), a(a) {}
    
    Color operator*(float scalar) const {
        return Color(
            static_cast<uint8_t>(std::min(255.0f, r * scalar)),
            static_cast<uint8_t>(std::min(255.0f, g * scalar)),
            static_cast<uint8_t>(std::min(255.0f, b * scalar)),
            a
        );
    }
    
    Color operator+(const Color& other) const {
        return Color(
            std::min(255, r + other.r),
            std::min(255, g + other.g),
            std::min(255, b + other.b),
            std::min(255, a + other.a)
        );
    }
};

struct Vector2 {
    float x, y;
    Vector2(float x = 0, float y = 0) : x(x), y(y) {}
    
    float length() const { return sqrt(x*x + y*y); }
    Vector2 normalized() const { 
        float len = length();
        if (len > 0) return Vector2(x/len, y/len);
        return Vector2(0, 0);
    }
    
    Vector2 operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }
    
    bool operator==(const Vector2& other) const {
        return x == other.x && y == other.y;
    }
};

struct Vector3 {
    float x, y, z;
    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    float length() const { return sqrt(x*x + y*y + z*z); }
    Vector3 normalized() const { 
        float len = length();
        if (len > 0.0001f) return Vector3(x/len, y/len, z/len);
        return Vector3(0, 0, 0);
    }
    
    bool operator==(const Vector3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std {
    template<>
    struct hash<Vector2> {
        size_t operator()(const Vector2& v) const {
            return hash<float>()(v.x) ^ (hash<float>()(v.y) << 1);
        }
    };
}

// ==================== FORWARD DECLARATIONS ====================
class Transform;
class Material;
class GameObject;

// ==================== COMPONENT BASE ====================
class Component {
public:
    virtual ~Component() = default;
    virtual void update() {}
    virtual void render() {}
    
    GameObject* gameObject = nullptr;
    Transform* GetTransform();
};

// ==================== TRANSFORM WITH HIERARCHY ====================
class Transform : public Component {
private:
    GameObject* parent = nullptr;
    std::vector<GameObject*> children;
    
    // Кэшированные мировые преобразования
    mutable Vector3 worldPosition;
    mutable Vector3 worldRotation;
    mutable Vector3 worldScale;
    mutable bool worldTransformDirty = true;
    
public:
    Vector3 position;
    Vector3 rotation;
    Vector3 scale;
    
    Transform(Vector3 pos = Vector3(0,0,0)) : position(pos), rotation(0,0,0), scale(1,1,1) {}
    
    void SetParent(GameObject* newParent);
    void AddChild(GameObject* child);
    void RemoveChild(GameObject* child);
    
    GameObject* GetParent() const { return parent; }
    const std::vector<GameObject*>& GetChildren() const { return children; }
    
    // Мировые координаты с учетом иерархии
    Vector3 GetWorldPosition() const {
        UpdateWorldTransform();
        return worldPosition;
    }
    
    Vector3 GetWorldRotation() const {
        UpdateWorldTransform();
        return worldRotation;
    }
    
    Vector3 GetWorldScale() const {
        UpdateWorldTransform();
        return worldScale;
    }
    
    // Локальные координаты относительно родителя
    Vector3 GetLocalPosition() const { return position; }
    Vector3 GetLocalRotation() const { return rotation; }
    Vector3 GetLocalScale() const { return scale; }
    
    void SetWorldPosition(const Vector3& newPosition);
    void SetWorldRotation(const Vector3& newRotation);
    void SetWorldScale(const Vector3& newScale);
    
    // Преобразования с учетом иерархии
    Vector3 TransformPoint(const Vector3& point) const;
    Vector3 TransformDirection(const Vector3& direction) const;
    Vector3 InverseTransformPoint(const Vector3& point) const;
    Vector3 InverseTransformDirection(const Vector3& direction) const;
    
private:
    void UpdateWorldTransform() const;
    void MarkTransformDirty();
};

// ==================== OTHER COMPONENTS ====================
class BoxCollider : public Component {
public:
    Vector3 size;
    Vector3 center;
    bool isTrigger;
    
    BoxCollider(Vector3 s = Vector3(1,1,1), Vector3 c = Vector3(0,0,0)) 
        : size(s), center(c), isTrigger(false) {}
};

class Rigidbody : public Component {
public:
    Vector3 velocity;
    Vector3 angularVelocity;
    float mass;
    float drag;
    float angularDrag;
    bool useGravity;
    bool isKinematic;
    
    Rigidbody(float m = 1.0f) 
        : velocity(0,0,0), angularVelocity(0,0,0), mass(m), 
          drag(0.1f), angularDrag(0.05f), useGravity(true), isKinematic(false) {}
    
    void AddForce(Vector3 force) {
        if (!isKinematic) {
            velocity = velocity + force * (1.0f / mass);
        }
    }
    
    void AddTorque(Vector3 torque) {
        if (!isKinematic) {
            angularVelocity = angularVelocity + torque * (1.0f / mass);
        }
    }
};

// ==================== GAME OBJECT WITH HIERARCHY ====================
class GameObject {
private:
    std::vector<std::unique_ptr<Component>> components;
    
public:
    std::string name;
    Transform* transform;
    bool hasRigidbody;
    bool hasCollider;
    
    GameObject(std::string objName = "GameObject") : name(objName), hasRigidbody(false), hasCollider(false) {
        transform = new Transform();
        addComponent(std::unique_ptr<Transform>(transform));
    }
    
    ~GameObject() {
        // Удаляем из иерархии при уничтожении
        if (transform->GetParent()) {
            transform->GetParent()->RemoveChild(this);
        }
        
        // Удаляем всех детей
        auto children = transform->GetChildren();
        for (auto child : children) {
            child->transform->SetParent(nullptr);
        }
        
        components.clear();
    }
    
    template<typename T>
    T* addComponent(std::unique_ptr<T> component) {
        T* ptr = component.get();
        component->gameObject = this;
        components.push_back(std::move(component));
        
        if (dynamic_cast<Rigidbody*>(ptr)) hasRigidbody = true;
        if (dynamic_cast<BoxCollider*>(ptr)) hasCollider = true;
        
        return ptr;
    }
    
    template<typename T>
    T* getComponent() {
        for (auto& comp : components) {
            T* result = dynamic_cast<T*>(comp.get());
            if (result) return result;
        }
        return nullptr;
    }
    
    // Методы для работы с иерархией
    void SetParent(GameObject* newParent) {
        transform->SetParent(newParent);
    }
    
    GameObject* GetParent() const {
        return transform->GetParent();
    }
    
    const std::vector<GameObject*>& GetChildren() const {
        return transform->GetChildren();
    }
    
    void AddChild(GameObject* child) {
        transform->AddChild(child);
    }
    
    void RemoveChild(GameObject* child) {
        transform->RemoveChild(child);
    }
    
    // Рекурсивный поиск по имени
    GameObject* FindChild(const std::string& childName, bool recursive = true) {
        for (auto child : transform->GetChildren()) {
            if (child->name == childName) {
                return child;
            }
            if (recursive) {
                GameObject* result = child->FindChild(childName, true);
                if (result) return result;
            }
        }
        return nullptr;
    }
    
    // Рекурсивный поиск компонента
    template<typename T>
    T* GetComponentInChildren() {
        for (auto child : transform->GetChildren()) {
            T* component = child->getComponent<T>();
            if (component) return component;
            
            component = child->GetComponentInChildren<T>();
            if (component) return component;
        }
        return nullptr;
    }
    
    // Рекурсивный поиск всех компонентов
    template<typename T>
    std::vector<T*> GetComponentsInChildren() {
        std::vector<T*> results;
        
        for (auto child : transform->GetChildren()) {
            T* component = child->getComponent<T>();
            if (component) results.push_back(component);
            
            auto childComponents = child->GetComponentsInChildren<T>();
            results.insert(results.end(), childComponents.begin(), childComponents.end());
        }
        
        return results;
    }
    
    void update() {
        for (auto& comp : components) {
            comp->update();
        }
    }
};

// ==================== IMPLEMENT GETTRANSFORM ====================
Transform* Component::GetTransform() {
    return gameObject ? gameObject->transform : nullptr;
}

// ==================== IMPLEMENT TRANSFORM HIERARCHY ====================
void Transform::SetParent(GameObject* newParent) {
    if (parent == newParent) return;
    
    // Удаляем из старого родителя
    if (parent) {
        parent->transform->RemoveChild(gameObject);
    }
    
    parent = newParent;
    
    // Добавляем к новому родителю
    if (parent) {
        parent->transform->AddChild(gameObject);
    }
    
    MarkTransformDirty();
}

void Transform::AddChild(GameObject* child) {
    if (std::find(children.begin(), children.end(), child) == children.end()) {
        children.push_back(child);
        child->transform->MarkTransformDirty();
    }
}

void Transform::RemoveChild(GameObject* child) {
    auto it = std::find(children.begin(), children.end(), child);
    if (it != children.end()) {
        children.erase(it);
        child->transform->MarkTransformDirty();
    }
}

void Transform::UpdateWorldTransform() const {
    if (!worldTransformDirty) return;
    
    if (parent) {
        Transform* parentTransform = parent->transform;
        parentTransform->UpdateWorldTransform();
        
        // Комбинируем преобразования: world = parent * local
        worldPosition = parentTransform->TransformPoint(position);
        worldRotation = parentTransform->GetWorldRotation() + rotation;
        worldScale = Vector3(
            parentTransform->GetWorldScale().x * scale.x,
            parentTransform->GetWorldScale().y * scale.y,
            parentTransform->GetWorldScale().z * scale.z
        );
    } else {
        worldPosition = position;
        worldRotation = rotation;
        worldScale = scale;
    }
    
    worldTransformDirty = false;
}

void Transform::MarkTransformDirty() {
    worldTransformDirty = true;
    for (auto child : children) {
        child->transform->MarkTransformDirty();
    }
}

void Transform::SetWorldPosition(const Vector3& newPosition) {
    if (parent) {
        // Конвертируем мировые координаты в локальные относительно родителя
        position = parent->transform->InverseTransformPoint(newPosition);
    } else {
        position = newPosition;
    }
    MarkTransformDirty();
}

void Transform::SetWorldRotation(const Vector3& newRotation) {
    if (parent) {
        rotation = newRotation - parent->transform->GetWorldRotation();
    } else {
        rotation = newRotation;
    }
    MarkTransformDirty();
}

void Transform::SetWorldScale(const Vector3& newScale) {
    if (parent) {
        Vector3 parentScale = parent->transform->GetWorldScale();
        scale = Vector3(
            newScale.x / parentScale.x,
            newScale.y / parentScale.y,
            newScale.z / parentScale.z
        );
    } else {
        scale = newScale;
    }
    MarkTransformDirty();
}

Vector3 Transform::TransformPoint(const Vector3& point) const {
    UpdateWorldTransform();
    
    // Применяем масштаб, поворот и позицию
    Vector3 scaledPoint = Vector3(
        point.x * worldScale.x,
        point.y * worldScale.y,
        point.z * worldScale.z
    );
    
    // Простое вращение (можно улучшить матрицами)
    Vector3 rotatedPoint = scaledPoint; // Упрощенная версия
    
    return rotatedPoint + worldPosition;
}

Vector3 Transform::TransformDirection(const Vector3& direction) const {
    UpdateWorldTransform();
    
    // Вращаем направление (упрощенно)
    Vector3 rotatedDir = direction; // Упрощенная версия
    
    return Vector3(
        rotatedDir.x * worldScale.x,
        rotatedDir.y * worldScale.y,
        rotatedDir.z * worldScale.z
    );
}

Vector3 Transform::InverseTransformPoint(const Vector3& point) const {
    UpdateWorldTransform();
    
    Vector3 localPoint = point - worldPosition;
    
    // Обратное масштабирование
    return Vector3(
        localPoint.x / worldScale.x,
        localPoint.y / worldScale.y,
        localPoint.z / worldScale.z
    );
}

Vector3 Transform::InverseTransformDirection(const Vector3& direction) const {
    UpdateWorldTransform();
    
    return Vector3(
        direction.x / worldScale.x,
        direction.y / worldScale.y,
        direction.z / worldScale.z
    );
}

// ==================== MATERIAL SYSTEM ====================
class Material {
public:
    Color albedo;
    float metallic;
    float roughness;
    float specular;
    
    Material(Color baseColor = Color(255, 255, 255))
        : albedo(baseColor), metallic(0.0f), roughness(0.5f), specular(0.5f) {}
};

// ==================== SHADER SYSTEM ====================
class Shader {
public:
    virtual ~Shader() = default;
    virtual Color CalculateColor(const Vector3& position, const Vector3& normal, 
                               const Vector3& viewDir, Material* material) = 0;
};

class LitShader : public Shader {
public:
    Color CalculateColor(const Vector3& position, const Vector3& normal, 
                        const Vector3& viewDir, Material* material) override {
        Vector3 lightDir = Vector3(0.5f, 1.0f, 0.5f).normalized();
        float diff = std::max(normal.dot(lightDir), 0.0f);
        
        Vector3 reflectDir = lightDir - normal * 2.0f * normal.dot(lightDir);
        float spec = pow(std::max(viewDir.dot(reflectDir), 0.0f), 32.0f);
        
        Color diffuse = material->albedo * (diff * 0.8f + 0.2f);
        Color specular = Color(255, 255, 255) * (spec * material->specular);
        
        return diffuse + specular;
    }
};

class UnlitShader : public Shader {
public:
    Color CalculateColor(const Vector3& position, const Vector3& normal, 
                        const Vector3& viewDir, Material* material) override {
        return material->albedo;
    }
};

class ToonShader : public Shader {
public:
    Color CalculateColor(const Vector3& position, const Vector3& normal, 
                        const Vector3& viewDir, Material* material) override {
        Vector3 lightDir = Vector3(0.5f, 1.0f, 0.5f).normalized();
        float diff = normal.dot(lightDir);
        
        if (diff > 0.8f) return material->albedo * 1.0f;
        else if (diff > 0.5f) return material->albedo * 0.7f;
        else if (diff > 0.2f) return material->albedo * 0.4f;
        else return material->albedo * 0.2f;
    }
};

// ==================== LIGHTING SYSTEM ====================
class Light : public Component {
public:
    enum Type { DIRECTIONAL, POINT, SPOT };
    
    Type type;
    Color color;
    float intensity;
    float range;
    Vector3 direction;
    
    Light(Type t = DIRECTIONAL, Color c = Color(255, 255, 255), float i = 1.0f)
        : type(t), color(c), intensity(i), range(10.0f), direction(0, -1, 0) {}
};

class LightingSystem {
private:
    std::vector<Light*> lights;
    
public:
    void AddLight(Light* light) {
        lights.push_back(light);
    }
    
    void RemoveLight(Light* light) {
        lights.erase(std::remove(lights.begin(), lights.end(), light), lights.end());
    }
    
    Color CalculateLighting(const Vector3& position, const Vector3& normal, 
                           const Vector3& viewDir, Material* material) {
        if (lights.empty()) {
            return material->albedo * 0.2f;
        }
        
        Color result(0, 0, 0);
        
        for (auto light : lights) {
            Vector3 lightDir;
            float attenuation = 1.0f;
            
            if (light->type == Light::DIRECTIONAL) {
                lightDir = light->direction.normalized();
            } else {
                Transform* lightTransform = light->GetTransform();
                if (lightTransform) {
                    lightDir = (lightTransform->GetWorldPosition() - position).normalized();
                    float distance = (lightTransform->GetWorldPosition() - position).length();
                    attenuation = 1.0f / (1.0f + 0.1f * distance + 0.01f * distance * distance);
                } else {
                    continue;
                }
            }
            
            float diff = std::max(normal.dot(lightDir), 0.0f);
            Color diffusePart = material->albedo * diff;
            Color diffuse = Color(
                static_cast<uint8_t>(light->color.r * diffusePart.r / 255.0f * light->intensity * attenuation),
                static_cast<uint8_t>(light->color.g * diffusePart.g / 255.0f * light->intensity * attenuation),
                static_cast<uint8_t>(light->color.b * diffusePart.b / 255.0f * light->intensity * attenuation)
            );
            
            Vector3 reflectDir = lightDir - normal * 2.0f * normal.dot(lightDir);
            float spec = pow(std::max(viewDir.dot(reflectDir), 0.0f), 32.0f);
            Color specular = Color(255, 255, 255) * (spec * material->specular * light->intensity * attenuation);
            
            result = result + diffuse + specular;
        }
        
        Color ambient = material->albedo * 0.1f;
        return ambient + result;
    }
    
    void Clear() {
        lights.clear();
    }
};

// ==================== OPTIMIZED MESH RENDERER ====================
class MeshRenderer : public Component {
private:
    std::vector<Vector3> vertices;
    std::vector<std::tuple<int, int, int>> triangles;
    std::vector<std::pair<int, int>> edges;
    std::shared_ptr<Material> material;
    std::shared_ptr<Shader> shader;
    
public:
    MeshRenderer(std::shared_ptr<Material> mat = std::make_shared<Material>(), 
                 std::shared_ptr<Shader> sh = std::make_shared<LitShader>()) 
        : material(mat), shader(sh) {
        createCubeMesh();
    }
    
private:
    void createCubeMesh() {
        vertices = {
            Vector3(-1, -1, -1), Vector3(1, -1, -1), Vector3(1, 1, -1), Vector3(-1, 1, -1),
            Vector3(-1, -1, 1), Vector3(1, -1, 1), Vector3(1, 1, 1), Vector3(-1, 1, 1)
        };
        
        triangles = {
            {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
            {3, 2, 6}, {3, 6, 7}, {0, 4, 5}, {0, 5, 1},
            {1, 5, 6}, {1, 6, 2}, {0, 3, 7}, {0, 7, 4}
        };
        
        edges = {
            {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4},
            {0,4}, {1,5}, {2,6}, {3,7}
        };
    }
    
public:
    void SetMaterial(std::shared_ptr<Material> newMaterial) {
        material = newMaterial;
    }
    
    void SetShader(std::shared_ptr<Shader> newShader) {
        shader = newShader;
    }
    
    std::shared_ptr<Material> GetMaterial() const { return material; }
    std::shared_ptr<Shader> GetShader() const { return shader; }
    
    const std::vector<Vector3>& getVertices() const { return vertices; }
    const std::vector<std::tuple<int, int, int>>& getTriangles() const { return triangles; }
    const std::vector<std::pair<int, int>>& getEdges() const { return edges; }
};

class Rotator : public Component {
public:
    float speed;
    Rotator(float s = 1.0f) : speed(s) {}
    
    void update() override {
        if (gameObject) {
            gameObject->transform->rotation.y += 0.01f * speed;
        }
    }
};

// ==================== OBJECT SELECTION SYSTEM ====================
class ObjectSelector : public Component {
private:
    bool isSelected = false;
    Color originalColor;
    Color selectedColor = Color(255, 255, 0);
    
public:
    void Select() {
        isSelected = true;
        if (gameObject) {
            MeshRenderer* mesh = gameObject->getComponent<MeshRenderer>();
            if (mesh) {
                originalColor = mesh->GetMaterial()->albedo;
                auto newMaterial = std::make_shared<Material>(*mesh->GetMaterial());
                newMaterial->albedo = selectedColor;
                mesh->SetMaterial(newMaterial);
            }
        }
    }
    
    void Deselect() {
        isSelected = false;
        if (gameObject) {
            MeshRenderer* mesh = gameObject->getComponent<MeshRenderer>();
            if (mesh) {
                auto newMaterial = std::make_shared<Material>(*mesh->GetMaterial());
                newMaterial->albedo = originalColor;
                mesh->SetMaterial(newMaterial);
            }
        }
    }
    
    bool IsSelected() const { return isSelected; }
};

// ==================== OBJECT POOLING SYSTEM ====================
class GameObjectPool {
private:
    std::vector<std::unique_ptr<GameObject>> pool;
    size_t activeCount = 0;
    const size_t MAX_POOL_SIZE = 50;
    std::vector<std::shared_ptr<Material>> materials;
    std::vector<std::shared_ptr<Shader>> shaders;
    
public:
    GameObjectPool() {
        // Create materials for pool
        materials = {
            std::make_shared<Material>(Color(255, 100, 100)),
            std::make_shared<Material>(Color(100, 255, 100)),
            std::make_shared<Material>(Color(100, 100, 255)),
            std::make_shared<Material>(Color(255, 215, 0))
        };
        
        // Create shaders for pool
        shaders = {
            std::make_shared<LitShader>(),
            std::make_shared<UnlitShader>(),
            std::make_shared<ToonShader>()
        };
    }
    
    void Initialize() {
        pool.reserve(MAX_POOL_SIZE);
        
        for (size_t i = 0; i < MAX_POOL_SIZE; i++) {
            auto obj = std::make_unique<GameObject>("PooledCube_" + std::to_string(i));
            
            // Add standard components
            auto material = materials[i % materials.size()];
            auto shader = shaders[i % shaders.size()];
            obj->addComponent(std::make_unique<MeshRenderer>(material, shader));
            obj->addComponent(std::make_unique<BoxCollider>());
            obj->addComponent(std::make_unique<Rigidbody>(1.0f + (rand() % 5)));
            obj->addComponent(std::make_unique<Rotator>(0.5f + (rand() % 100) * 0.01f));
            obj->addComponent(std::make_unique<ObjectSelector>());
            
            // Deactivate initially (position far behind camera)
            obj->transform->position = Vector3(-1000, -1000, -1000);
            obj->transform->scale = Vector3(0.8f, 0.8f, 0.8f);
            
            pool.push_back(std::move(obj));
        }
        
        std::cout << "Object Pool initialized with " << MAX_POOL_SIZE << " objects" << std::endl;
    }
    
    GameObject* GetObject() {
        if (activeCount >= pool.size()) {
            std::cout << "Object Pool is full! Cannot create more objects." << std::endl;
            return nullptr;
        }
        
        GameObject* obj = pool[activeCount].get();
        activeCount++;
        
        // Activate object - place in random position
        obj->transform->position = Vector3(
            (rand() % 20) - 10,
            (rand() % 10) + 5,
            (rand() % 20) - 10
        );
        
        // Reset physics
        Rigidbody* rb = obj->getComponent<Rigidbody>();
        if (rb) {
            rb->velocity = Vector3(0, 0, 0);
            rb->angularVelocity = Vector3(0, 0, 0);
        }
        
        // Random material
        MeshRenderer* mesh = obj->getComponent<MeshRenderer>();
        if (mesh) {
            auto newMaterial = materials[rand() % materials.size()];
            mesh->SetMaterial(newMaterial);
        }
        
        std::cout << "Object activated from pool. Active: " << activeCount << "/" << pool.size() << std::endl;
        return obj;
    }
    
    void ReturnObject(GameObject* obj) {
        // Удаляем из иерархии перед возвратом в пул
        obj->SetParent(nullptr);
        
        // Find object in pool and deactivate it
        for (size_t i = 0; i < activeCount; i++) {
            if (pool[i].get() == obj) {
                // Move to end of active and decrease counter
                std::swap(pool[i], pool[activeCount - 1]);
                activeCount--;
                
                // Deactivate object
                obj->transform->position = Vector3(-1000, -1000, -1000);
                
                std::cout << "Object returned to pool. Active: " << activeCount << "/" << pool.size() << std::endl;
                return;
            }
        }
    }
    
    void ReturnAllObjects() {
        for (size_t i = 0; i < activeCount; i++) {
            pool[i]->SetParent(nullptr);
            pool[i]->transform->position = Vector3(-1000, -1000, -1000);
        }
        activeCount = 0;
        std::cout << "All objects returned to pool." << std::endl;
    }
    
    std::vector<GameObject*> GetActiveObjects() {
        std::vector<GameObject*> result;
        result.reserve(activeCount);
        for (size_t i = 0; i < activeCount; i++) {
            result.push_back(pool[i].get());
        }
        return result;
    }
    
    size_t GetActiveCount() const { return activeCount; }
    size_t GetPoolSize() const { return pool.size(); }
    size_t GetAvailableCount() const { return pool.size() - activeCount; }
};

// ==================== OPTIMIZED CAMERA ====================
class Camera {
public:
    Vector3 position;
    Vector3 rotation;
    float moveSpeed;
    float lookSpeed;
    
    Camera() : position(0, 0, -5), rotation(0, 0, 0), moveSpeed(0.1f), lookSpeed(0.005f) {}
    
    void move(Vector3 direction) {
        position = position + direction * moveSpeed;
    }
    
    void look(float deltaX, float deltaY) {
        rotation.y += deltaX * lookSpeed;
        rotation.x += deltaY * lookSpeed;
        
        rotation.x = std::max(-3.14f/2, std::min(3.14f/2, rotation.x));
    }
    
    Vector3 getForward() const {
        return Vector3(
            sin(rotation.y) * cos(rotation.x),
            -sin(rotation.x),
            cos(rotation.y) * cos(rotation.x)
        );
    }
    
    Vector3 getRight() const {
        return Vector3(cos(rotation.y), 0, -sin(rotation.y));
    }
    
    Vector3 getViewDirection(const Vector3& worldPos) const {
        return (worldPos - position).normalized();
    }
    
    bool isInView(const Vector3& point, float radius = 1.0f) const {
        Vector3 viewDir = getViewDirection(point);
        float distance = (point - position).length();
        return distance < 50.0f && viewDir.z > 0.1f;
    }
};

// ==================== OPTIMIZED PHYSICS SYSTEM ====================
class PhysicsSystem {
private:
    Vector3 gravity;
    
    struct PhysicsObject {
        BoxCollider* collider;
        Rigidbody* rigidbody;
        Transform* transform;
        GameObject* gameObject;
    };
    
    std::vector<PhysicsObject> physicsObjects;
    std::unordered_map<Vector2, std::vector<GameObject*>> spatialGrid;
    const float CELL_SIZE = 8.0f; // Increased cell size
    
public:
    PhysicsSystem() : gravity(0, 9.81f, 0) {}
    
    void RegisterPhysicsObject(GameObject* obj) {
        BoxCollider* col = obj->getComponent<BoxCollider>();
        Rigidbody* rb = obj->getComponent<Rigidbody>();
        
        if (col && rb) {
            physicsObjects.push_back({col, rb, obj->transform, obj});
        }
    }
    
    void UpdateSpatialPartition() {
        spatialGrid.clear();
        
        for (auto& physObj : physicsObjects) {
            if (!physObj.collider || !physObj.transform) continue;
            
            Vector3 pos = physObj.transform->GetWorldPosition();
            int gridX = static_cast<int>(pos.x / CELL_SIZE);
            int gridZ = static_cast<int>(pos.z / CELL_SIZE);
            
            spatialGrid[Vector2(gridX, gridZ)].push_back(physObj.gameObject);
        }
    }
    
    // Simplified collision check - use spheres instead of AABB
    bool CheckCollisionFast(Transform* t1, BoxCollider* c1, Transform* t2, BoxCollider* c2) {
        Vector3 pos1 = t1->GetWorldPosition();
        Vector3 pos2 = t2->GetWorldPosition();
        
        // Use spherical collisions for speed
        float radius1 = std::max({c1->size.x, c1->size.y, c1->size.z}) * 0.5f;
        float radius2 = std::max({c2->size.x, c2->size.y, c2->size.z}) * 0.5f;
        
        float distance = (pos1 - pos2).length();
        return distance < (radius1 + radius2);
    }
    
    void ResolveCollisionSimple(Rigidbody* rb1, Rigidbody* rb2, Vector3 collisionNormal) {
        if (rb1->isKinematic && rb2->isKinematic) return;
        
        // Simplified collision resolution
        Vector3 relativeVelocity = rb2->velocity - rb1->velocity;
        float velocityAlongNormal = relativeVelocity.dot(collisionNormal);
        
        if (velocityAlongNormal > 0) return;
        
        float restitution = 0.5f; // Reduced elasticity for stability
        float j = -(1 + restitution) * velocityAlongNormal;
        
        // Simplified mass
        float invMass1 = rb1->isKinematic ? 0 : 1.0f / rb1->mass;
        float invMass2 = rb2->isKinematic ? 0 : 1.0f / rb2->mass;
        
        j /= (invMass1 + invMass2);
        
        Vector3 impulse = collisionNormal * j;
        
        if (!rb1->isKinematic) {
            rb1->velocity = rb1->velocity - impulse * invMass1;
        }
        if (!rb2->isKinematic) {
            rb2->velocity = rb2->velocity + impulse * invMass2;
        }
    }
    
    void CheckCollisionsInCell(const std::vector<GameObject*>& objects) {
        // Limit number of checks per cell
        const int MAX_CHECKS_PER_CELL = 20;
        int checks = 0;
        
        for (int i = 0; i < objects.size() && checks < MAX_CHECKS_PER_CELL; i++) {
            BoxCollider* col1 = objects[i]->getComponent<BoxCollider>();
            Rigidbody* rb1 = objects[i]->getComponent<Rigidbody>();
            
            if (!col1) continue;
            
            for (int j = i + 1; j < objects.size() && checks < MAX_CHECKS_PER_CELL; j++) {
                BoxCollider* col2 = objects[j]->getComponent<BoxCollider>();
                Rigidbody* rb2 = objects[j]->getComponent<Rigidbody>();
                
                if (!col2) continue;
                
                checks++;
                if (CheckCollisionFast(objects[i]->transform, col1, objects[j]->transform, col2)) {
                    Vector3 collisionNormal = (objects[j]->transform->GetWorldPosition() - objects[i]->transform->GetWorldPosition()).normalized();
                    
                    if (!col1->isTrigger && !col2->isTrigger) {
                        ResolveCollisionSimple(rb1, rb2, collisionNormal);
                    }
                }
            }
        }
    }
    
    void CheckCollisionsWithNeighbors(const Vector2& cell, const std::vector<GameObject*>& objects) {
        // Only nearest neighbors (no diagonals)
        static const std::vector<Vector2> neighborOffsets = {
            Vector2(1, 0), Vector2(-1, 0), Vector2(0, 1), Vector2(0, -1)
        };
        
        for (auto& offset : neighborOffsets) {
            Vector2 neighborCell = Vector2(cell.x + offset.x, cell.y + offset.y);
            
            if (spatialGrid.find(neighborCell) != spatialGrid.end()) {
                const auto& neighborObjects = spatialGrid[neighborCell];
                
                for (auto obj1 : objects) {
                    for (auto obj2 : neighborObjects) {
                        if (obj1 == obj2) continue;
                        
                        BoxCollider* col1 = obj1->getComponent<BoxCollider>();
                        Rigidbody* rb1 = obj1->getComponent<Rigidbody>();
                        BoxCollider* col2 = obj2->getComponent<BoxCollider>();
                        Rigidbody* rb2 = obj2->getComponent<Rigidbody>();
                        
                        if (!col1 || !col2) continue;
                        
                        if (CheckCollisionFast(obj1->transform, col1, obj2->transform, col2)) {
                            Vector3 collisionNormal = (obj2->transform->GetWorldPosition() - obj1->transform->GetWorldPosition()).normalized();
                            
                            if (!col1->isTrigger && !col2->isTrigger) {
                                ResolveCollisionSimple(rb1, rb2, collisionNormal);
                            }
                        }
                    }
                }
            }
        }
    }
    
    void Update(std::vector<GameObject*>& objects, float deltaTime) {
        const float fixedDeltaTime = 1.0f / 60.0f;
        
        // Optimization: update physics only for active objects
        int activePhysicsObjects = 0;
        for (auto& physObj : physicsObjects) {
            if (physObj.rigidbody && !physObj.rigidbody->isKinematic) {
                activePhysicsObjects++;
                
                if (physObj.rigidbody->useGravity) {
                    physObj.rigidbody->velocity = physObj.rigidbody->velocity + gravity * fixedDeltaTime;
                }
                
                // Simplified drag
                physObj.rigidbody->velocity = physObj.rigidbody->velocity * 0.99f;
                physObj.rigidbody->angularVelocity = physObj.rigidbody->angularVelocity * 0.95f;
                
                // Используем локальные координаты для перемещения
                physObj.transform->position = physObj.transform->position + physObj.rigidbody->velocity * fixedDeltaTime;
                physObj.transform->rotation = physObj.transform->rotation + physObj.rigidbody->angularVelocity * fixedDeltaTime;
            }
        }
        
        // Update spatial partitioning only if there are active objects
        if (activePhysicsObjects > 0) {
            UpdateSpatialPartition();
            
            for (auto& [cell, cellObjects] : spatialGrid) {
                if (cellObjects.size() > 1) {
                    CheckCollisionsInCell(cellObjects);
                }
                if (cellObjects.size() > 0) {
                    CheckCollisionsWithNeighbors(cell, cellObjects);
                }
            }
        }
    }
    
    void Clear() {
        physicsObjects.clear();
        spatialGrid.clear();
    }
};

// ==================== OPTIMIZED RENDER SYSTEM ====================
class RenderSystem {
private:
    SDL_Renderer* sdlRenderer;
    SDL_Texture* texture;
    uint32_t* pixels;
    float* zBuffer;
    int pitch;
    LightingSystem lightingSystem;
    TTF_Font* font;
    TTF_Font* smallFont;
    
    struct RenderItem {
        Transform* transform;
        MeshRenderer* mesh;
        float distanceToCamera;
        
        RenderItem(Transform* t, MeshRenderer* m, float dist) 
            : transform(t), mesh(m), distanceToCamera(dist) {}
        
        bool operator<(const RenderItem& other) const {
            return distanceToCamera > other.distanceToCamera;
        }
    };
    
    std::vector<RenderItem> renderQueue;
    
    bool pointInTriangleFast(int x, int y, 
                           int x1, int y1, int x2, int y2, int x3, int y3,
                           int dx12, int dx23, int dx31, 
                           int dy12, int dy23, int dy31) const {
        int w1 = (x - x2) * dy12 - dx12 * (y - y2);
        int w2 = (x - x3) * dy23 - dx23 * (y - y3);
        int w3 = (x - x1) * dy31 - dx31 * (y - y1);
        
        return (w1 >= 0 && w2 >= 0 && w3 >= 0) || (w1 <= 0 && w2 <= 0 && w3 <= 0);
    }
    
    float interpolateZ(int x, int y, 
                      int x1, int y1, float z1,
                      int x2, int y2, float z2,
                      int x3, int y3, float z3) const {
        float area = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
        if (fabs(area) < 0.001f) return z1;
        
        float w1 = ((x2 - x) * (y3 - y) - (x3 - x) * (y2 - y)) / area;
        float w2 = ((x3 - x) * (y1 - y) - (x1 - x) * (y3 - y)) / area;
        float w3 = 1.0f - w1 - w2;
        
        return w1 * z1 + w2 * z2 + w3 * z3;
    }
    
public:
    RenderSystem(SDL_Window* window) {
        sdlRenderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        texture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_RGBA32,
                                   SDL_TEXTUREACCESS_STREAMING, 
                                   SCREEN_WIDTH, SCREEN_HEIGHT);
        pixels = new uint32_t[SCREEN_WIDTH * SCREEN_HEIGHT];
        zBuffer = new float[SCREEN_WIDTH * SCREEN_HEIGHT];
        pitch = SCREEN_WIDTH * sizeof(uint32_t);
        
        // Initialize TTF
        if (TTF_Init() == -1) {
            std::cout << "TTF_Init failed: " << TTF_GetError() << std::endl;
            font = nullptr;
            smallFont = nullptr;
            return;
        }
        
        // Try to load different fonts
        const char* fontPaths[] = {
            "arial.ttf",
            "DejaVuSans.ttf",
            "LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            nullptr
        };
        
        font = nullptr;
        for (int i = 0; fontPaths[i] && !font; i++) {
            font = TTF_OpenFont(fontPaths[i], 24);
            if (font) {
                std::cout << "Loaded font: " << fontPaths[i] << std::endl;
            }
        }
        
        smallFont = nullptr;
        for (int i = 0; fontPaths[i] && !smallFont; i++) {
            smallFont = TTF_OpenFont(fontPaths[i], 16);
            if (smallFont) {
                std::cout << "Loaded small font: " << fontPaths[i] << std::endl;
            }
        }
        
        // If no fonts loaded, use default
        if (!font) {
            std::cout << "Using default font" << std::endl;
            font = TTF_OpenFontIndex(nullptr, 24, 0);
        }
        if (!smallFont) {
            smallFont = TTF_OpenFontIndex(nullptr, 16, 0);
        }
    }
    
    ~RenderSystem() {
        delete[] pixels;
        delete[] zBuffer;
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(sdlRenderer);
        if (font) TTF_CloseFont(font);
        if (smallFont) TTF_CloseFont(smallFont);
        TTF_Quit();
    }
    
    void AddLight(Light* light) {
        lightingSystem.AddLight(light);
    }
    
    void clear(const Color& color) {
        uint32_t clearColor = (color.a << 24) | (color.b << 16) | (color.g << 8) | color.r;
        std::fill(pixels, pixels + SCREEN_WIDTH * SCREEN_HEIGHT, clearColor);
        std::fill(zBuffer, zBuffer + SCREEN_WIDTH * SCREEN_HEIGHT, std::numeric_limits<float>::max());
    }
    
    void addToRenderQueue(Transform* transform, MeshRenderer* mesh, const Camera& camera) {
        if (!transform || !mesh) return;
        
        // Optimization: skip objects far from camera
        if (!camera.isInView(transform->GetWorldPosition(), 2.0f)) {
            return;
        }
        
        float distance = (transform->GetWorldPosition() - camera.position).length();
        renderQueue.emplace_back(transform, mesh, distance);
    }
    
    void renderAllFromQueue(const Camera& camera) {
        std::sort(renderQueue.begin(), renderQueue.end());
        
        for (auto& item : renderQueue) {
            renderMeshOptimized(item.transform, item.mesh, camera);
        }
        
        renderQueue.clear();
    }
    
    void setPixel(int x, int y, float z, const Color& color) {
        if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
            if (z < zBuffer[y * SCREEN_WIDTH + x]) {
                zBuffer[y * SCREEN_WIDTH + x] = z;
                pixels[y * SCREEN_WIDTH + x] = (color.a << 24) | (color.b << 16) | (color.g << 8) | color.r;
            }
        }
    }
    
    void setPixelUI(int x, int y, const Color& color) {
        if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
            pixels[y * SCREEN_WIDTH + x] = (color.a << 24) | (color.b << 16) | (color.g << 8) | color.r;
        }
    }
    
    void drawRect(int x, int y, int w, int h, const Color& color) {
        for (int i = x; i < x + w && i < SCREEN_WIDTH; i++) {
            for (int j = y; j < y + h && j < SCREEN_HEIGHT; j++) {
                setPixelUI(i, j, color);
            }
        }
    }
    
    void drawText(const std::string& text, int x, int y, const Color& color, bool small = false) {
        TTF_Font* currentFont = small ? smallFont : font;
        if (!currentFont) {
            // Fallback: draw simple rectangles for letters
            drawSimpleText(text, x, y, color);
            return;
        }
        
        SDL_Color sdlColor = {color.r, color.g, color.b, 255};
        SDL_Surface* surface = TTF_RenderText_Solid(currentFont, text.c_str(), sdlColor);
        if (!surface) {
            drawSimpleText(text, x, y, color);
            return;
        }
        
        // Convert surface to our pixel format
        SDL_LockSurface(surface);
        uint8_t* surfacePixels = (uint8_t*)surface->pixels;
        
        for (int i = 0; i < surface->w && (x + i) < SCREEN_WIDTH; i++) {
            for (int j = 0; j < surface->h && (y + j) < SCREEN_HEIGHT; j++) {
                int srcIndex = j * surface->pitch + i * surface->format->BytesPerPixel;
                uint8_t r = surfacePixels[srcIndex + (surface->format->Rshift >> 3)];
                uint8_t g = surfacePixels[srcIndex + (surface->format->Gshift >> 3)];
                uint8_t b = surfacePixels[srcIndex + (surface->format->Bshift >> 3)];
                
                // If pixel is not black (text color), draw it
                if (r > 10 || g > 10 || b > 10) {
                    setPixelUI(x + i, y + j, color);
                }
            }
        }
        
        SDL_UnlockSurface(surface);
        SDL_FreeSurface(surface);
    }
    
    void drawSimpleText(const std::string& text, int x, int y, const Color& color) {
        // Simple fallback: draw rectangles for each character
        for (size_t i = 0; i < text.length(); i++) {
            int charX = x + i * 8;
            if (charX >= SCREEN_WIDTH) break;
            
            // Simple character representation (very basic)
            switch (text[i]) {
                case 'A': case 'a':
                    drawRect(charX, y, 6, 1, color);
                    drawRect(charX, y+3, 6, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+5, y, 1, 7, color);
                    break;
                case 'B': case 'b':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y+1, 1, 2, color);
                    drawRect(charX+4, y+4, 1, 2, color);
                    break;
                case 'C': case 'c':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    break;
                case 'D': case 'd':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y+1, 1, 5, color);
                    break;
                case 'E': case 'e':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    break;
                case 'F': case 'f':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    break;
                case 'G': case 'g':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y+3, 1, 4, color);
                    drawRect(charX+2, y+3, 3, 1, color);
                    break;
                case 'H': case 'h':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX, y+3, 5, 1, color);
                    break;
                case 'I': case 'i':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX+2, y, 1, 7, color);
                    break;
                case 'J': case 'j':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX+4, y, 1, 6, color);
                    drawRect(charX, y+6, 4, 1, color);
                    drawRect(charX, y+4, 1, 1, color);
                    break;
                case 'K': case 'k':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 3, color);
                    drawRect(charX+3, y+3, 1, 1, color);
                    drawRect(charX+2, y+4, 1, 1, color);
                    drawRect(charX+1, y+5, 1, 1, color);
                    drawRect(charX, y+6, 1, 1, color);
                    break;
                case 'L': case 'l':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX, y+6, 5, 1, color);
                    break;
                case 'M': case 'm':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX+1, y+1, 1, 1, color);
                    drawRect(charX+2, y+2, 1, 1, color);
                    drawRect(charX+3, y+1, 1, 1, color);
                    break;
                case 'N': case 'n':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX+1, y+1, 1, 1, color);
                    drawRect(charX+2, y+2, 1, 1, color);
                    drawRect(charX+3, y+3, 1, 1, color);
                    break;
                case 'O': case 'o':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    break;
                case 'P': case 'p':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 2, color);
                    break;
                case 'Q': case 'q':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX+3, y+5, 1, 1, color);
                    drawRect(charX+2, y+6, 1, 1, color);
                    break;
                case 'R': case 'r':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 2, color);
                    drawRect(charX+1, y+4, 1, 1, color);
                    drawRect(charX+2, y+5, 1, 1, color);
                    drawRect(charX+3, y+6, 1, 1, color);
                    break;
                case 'S': case 's':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y+1, 1, 2, color);
                    drawRect(charX+4, y+4, 1, 2, color);
                    break;
                case 'T': case 't':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX+2, y, 1, 7, color);
                    break;
                case 'U': case 'u':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX, y+6, 5, 1, color);
                    break;
                case 'V': case 'v':
                    drawRect(charX, y, 1, 6, color);
                    drawRect(charX+4, y, 1, 6, color);
                    drawRect(charX+1, y+6, 1, 1, color);
                    drawRect(charX+2, y+5, 1, 1, color);
                    drawRect(charX+3, y+6, 1, 1, color);
                    break;
                case 'W': case 'w':
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX+2, y+5, 1, 2, color);
                    drawRect(charX+1, y+3, 1, 1, color);
                    drawRect(charX+3, y+3, 1, 1, color);
                    break;
                case 'X': case 'x':
                    drawRect(charX, y, 1, 2, color);
                    drawRect(charX+4, y, 1, 2, color);
                    drawRect(charX+1, y+2, 1, 1, color);
                    drawRect(charX+3, y+2, 1, 1, color);
                    drawRect(charX+2, y+3, 1, 1, color);
                    drawRect(charX+1, y+4, 1, 1, color);
                    drawRect(charX+3, y+4, 1, 1, color);
                    drawRect(charX, y+5, 1, 2, color);
                    drawRect(charX+4, y+5, 1, 2, color);
                    break;
                case 'Y': case 'y':
                    drawRect(charX, y, 1, 3, color);
                    drawRect(charX+4, y, 1, 3, color);
                    drawRect(charX+2, y+3, 1, 4, color);
                    drawRect(charX+1, y+2, 1, 1, color);
                    drawRect(charX+3, y+2, 1, 1, color);
                    break;
                case 'Z': case 'z':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 2, color);
                    drawRect(charX+3, y+3, 1, 1, color);
                    drawRect(charX+2, y+4, 1, 1, color);
                    drawRect(charX+1, y+5, 1, 1, color);
                    break;
                case '0':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX+1, y+1, 3, 1, color);
                    drawRect(charX+1, y+5, 3, 1, color);
                    break;
                case '1':
                    drawRect(charX+2, y, 1, 7, color);
                    drawRect(charX+1, y+1, 1, 1, color);
                    drawRect(charX, y+2, 1, 1, color);
                    drawRect(charX, y+6, 4, 1, color);
                    break;
                case '2':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 2, color);
                    drawRect(charX, y+4, 1, 2, color);
                    break;
                case '3':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 5, color);
                    break;
                case '4':
                    drawRect(charX, y, 1, 4, color);
                    drawRect(charX+4, y, 1, 7, color);
                    drawRect(charX, y+3, 5, 1, color);
                    break;
                case '5':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y+1, 1, 2, color);
                    drawRect(charX+4, y+4, 1, 2, color);
                    break;
                case '6':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y+4, 1, 2, color);
                    break;
                case '7':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX+4, y+1, 1, 6, color);
                    break;
                case '8':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y, 1, 7, color);
                    drawRect(charX+4, y, 1, 7, color);
                    break;
                case '9':
                    drawRect(charX, y, 5, 1, color);
                    drawRect(charX, y+3, 5, 1, color);
                    drawRect(charX, y+6, 5, 1, color);
                    drawRect(charX, y+1, 1, 2, color);
                    drawRect(charX+4, y, 1, 7, color);
                    break;
                case ':':
                    drawRect(charX+2, y+2, 1, 1, color);
                    drawRect(charX+2, y+4, 1, 1, color);
                    break;
                case '/':
                    drawRect(charX+4, y, 1, 2, color);
                    drawRect(charX+3, y+2, 1, 1, color);
                    drawRect(charX+2, y+3, 1, 1, color);
                    drawRect(charX+1, y+4, 1, 1, color);
                    drawRect(charX, y+5, 1, 2, color);
                    break;
                case ' ':
                    // Space - do nothing
                    break;
                default:
                    // Unknown character - draw a box
                    drawRect(charX, y, 5, 7, color);
                    break;
            }
        }
    }
    
    void drawLine(int x1, int y1, float z1, int x2, int y2, float z2, const Color& color) {
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        float zStep = (z2 - z1) / std::max(1, std::max(dx, dy));
        float currentZ = z1;
        
        while (true) {
            setPixel(x1, y1, currentZ, color);
            if (x1 == x2 && y1 == y2) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
            currentZ += zStep;
        }
    }
    
    void fillTriangleOptimized(int x1, int y1, float z1, int x2, int y2, float z2, 
                              int x3, int y3, float z3, const Color& color) {
        int minX = std::max(0, std::min({x1, x2, x3}));
        int maxX = std::min(SCREEN_WIDTH - 1, std::max({x1, x2, x3}));
        int minY = std::max(0, std::min({y1, y2, y3}));
        int maxY = std::min(SCREEN_HEIGHT - 1, std::max({y1, y2, y3}));
        
        int dx12 = x1 - x2, dx23 = x2 - x3, dx31 = x3 - x1;
        int dy12 = y1 - y2, dy23 = y2 - y3, dy31 = y3 - y1;
        
        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                if (pointInTriangleFast(x, y, x1, y1, x2, y2, x3, y3, 
                                      dx12, dx23, dx31, dy12, dy23, dy31)) {
                    float z = interpolateZ(x, y, x1, y1, z1, x2, y2, z2, x3, y3, z3);
                    setPixel(x, y, z, color);
                }
            }
        }
    }
    
    Vector3 project(const Vector3& point, const Camera& camera) {
        Vector3 translated = point - camera.position;
        
        Vector3 rotatedY = Vector3(
            translated.x * cos(camera.rotation.y) + translated.z * sin(camera.rotation.y),
            translated.y,
            -translated.x * sin(camera.rotation.y) + translated.z * cos(camera.rotation.y)
        );
        
        Vector3 rotatedX = Vector3(
            rotatedY.x,
            rotatedY.y * cos(camera.rotation.x) - rotatedY.z * sin(camera.rotation.x),
            rotatedY.y * sin(camera.rotation.x) + rotatedY.z * cos(camera.rotation.x)
        );
        
        float scale = 200.0f;
        float z = rotatedX.z;
        if (z <= 0.1f) z = 0.1f;
        
        float factor = scale / z;
        return Vector3(
            rotatedX.x * factor + SCREEN_WIDTH / 2,
            rotatedX.y * factor + SCREEN_HEIGHT / 2,
            z
        );
    }
    
private:
    Vector3 transformVertex(const Vector3& vertex, Transform* transform) {
        Vector3 result = vertex;
        result.x *= transform->GetWorldScale().x;
        result.y *= transform->GetWorldScale().y;
        result.z *= transform->GetWorldScale().z;
        return result + transform->GetWorldPosition();
    }
    
    void renderMeshOptimized(Transform* transform, MeshRenderer* mesh, const Camera& camera) {
        const auto& vertices = mesh->getVertices();
        const auto& triangles = mesh->getTriangles();
        const auto& edges = mesh->getEdges();
        auto material = mesh->GetMaterial();
        auto shader = mesh->GetShader();
        
        std::vector<Vector3> projectedVertices;
        projectedVertices.reserve(vertices.size());
        
        for (const auto& vertex : vertices) {
            Vector3 worldVertex = transformVertex(vertex, transform);
            projectedVertices.push_back(project(worldVertex, camera));
        }
        
        for (const auto& triangle : triangles) {
            int i1 = std::get<0>(triangle);
            int i2 = std::get<1>(triangle);
            int i3 = std::get<2>(triangle);
            
            Vector3& p1 = projectedVertices[i1];
            Vector3& p2 = projectedVertices[i2];
            Vector3& p3 = projectedVertices[i3];
            
            if (p1.z <= 0 || p2.z <= 0 || p3.z <= 0) continue;
            
            Vector3 worldV1 = transformVertex(vertices[i1], transform);
            Vector3 worldV2 = transformVertex(vertices[i2], transform);
            Vector3 worldV3 = transformVertex(vertices[i3], transform);
            
            Vector3 edge1 = worldV2 - worldV1;
            Vector3 edge2 = worldV3 - worldV1;
            Vector3 normal = edge1.cross(edge2).normalized();
            
            Vector3 faceCenter = (worldV1 + worldV2 + worldV3) * (1.0f / 3.0f);
            Vector3 viewDir = camera.getViewDirection(faceCenter);
            
            Color faceColor = shader->CalculateColor(faceCenter, normal, viewDir, material.get());
            
            fillTriangleOptimized(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z, faceColor);
        }
        
        for (const auto& edge : edges) {
            Vector3& p1 = projectedVertices[edge.first];
            Vector3& p2 = projectedVertices[edge.second];
            
            if (p1.z > 0 && p2.z > 0) {
                drawLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, Color(0, 0, 0));
            }
        }
    }
    
public:
    // Рендеринг связей иерархии
    void renderHierarchyConnections(const std::vector<GameObject*>& objects, const Camera& camera) {
        for (auto obj : objects) {
            auto children = obj->GetChildren();
            Vector3 parentPos = obj->transform->GetWorldPosition();
            Vector3 parentScreenPos = project(parentPos, camera);
            
            for (auto child : children) {
                Vector3 childPos = child->transform->GetWorldPosition();
                Vector3 childScreenPos = project(childPos, camera);
                
                if (parentScreenPos.z > 0 && childScreenPos.z > 0) {
                    drawLine(parentScreenPos.x, parentScreenPos.y, parentScreenPos.z,
                            childScreenPos.x, childScreenPos.y, childScreenPos.z,
                            Color(0, 255, 0, 150));
                }
            }
        }
    }
    
    bool raycast(const Vector2& screenPos, Transform* transform, MeshRenderer* mesh, 
                const Camera& camera, Vector3& hitPoint) {
        const auto& vertices = mesh->getVertices();
        const auto& triangles = mesh->getTriangles();
        
        Vector3 rayDir = screenToWorld(screenPos, camera);
        Vector3 rayOrigin = camera.position;
        
        for (const auto& triangle : triangles) {
            int i1 = std::get<0>(triangle);
            int i2 = std::get<1>(triangle);
            int i3 = std::get<2>(triangle);
            
            Vector3 v1 = transformVertex(vertices[i1], transform);
            Vector3 v2 = transformVertex(vertices[i2], transform);
            Vector3 v3 = transformVertex(vertices[i3], transform);
            
            if (rayTriangleIntersect(rayOrigin, rayDir, v1, v2, v3, hitPoint)) {
                return true;
            }
        }
        
        return false;
    }
    
private:
    Vector3 screenToWorld(const Vector2& screenPos, const Camera& camera) {
    float x = (screenPos.x - SCREEN_WIDTH / 2.0f) / 200.0f;
    float y = (SCREEN_HEIGHT / 2.0f - screenPos.y) / 200.0f;
    float z = 1.0f;

    Vector3 rayDirCam = Vector3(x, y, z).normalized();

    float cosX = cos(-camera.rotation.x);
    float sinX = sin(-camera.rotation.x);
    Vector3 rotatedX = Vector3(
        rayDirCam.x,
        rayDirCam.y * cosX - rayDirCam.z * sinX,
        rayDirCam.y * sinX + rayDirCam.z * cosX
    );

    float cosY = cos(-camera.rotation.y);
    float sinY = sin(-camera.rotation.y);
    Vector3 rotatedY = Vector3(
        rotatedX.x * cosY + rotatedX.z * sinY,
        rotatedX.y,
        -rotatedX.x * sinY + rotatedX.z * cosY
    );

    return rotatedY.normalized();
}
    
    bool rayTriangleIntersect(const Vector3& origin, const Vector3& dir,
                             const Vector3& v0, const Vector3& v1, const Vector3& v2,
                             Vector3& hitPoint) {
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 h = dir.cross(e2);
        float a = e1.dot(h);
        
        if (a > -0.00001f && a < 0.00001f)
            return false;
        
        float f = 1.0f / a;
        Vector3 s = origin - v0;
        float u = f * s.dot(h);
        
        if (u < 0.0f || u > 1.0f)
            return false;
        
        Vector3 q = s.cross(e1);
        float v = f * dir.dot(q);
        
        if (v < 0.0f || u + v > 1.0f)
            return false;
        
        float t = f * e2.dot(q);
        
        if (t > 0.00001f) {
            hitPoint = origin + dir * t;
            return true;
        }
        
        return false;
    }
    
public:
    void present() {
        SDL_UpdateTexture(texture, NULL, pixels, pitch);
        SDL_RenderCopy(sdlRenderer, texture, NULL, NULL);
        SDL_RenderPresent(sdlRenderer);
    }
};

// ==================== SCENE WITH HIERARCHY SUPPORT ====================
class Scene {
private:
    GameObjectPool objectPool;
    GameObject* selectedObject = nullptr;
    std::vector<std::unique_ptr<GameObject>> sceneObjects; // Все объекты сцены
    GameObject* sceneRoot; // Корневой объект всей сцены
    
public:
    void Initialize() {
        objectPool.Initialize();
        
        // Создаем корневой объект сцены
        sceneRoot = new GameObject("Scene");
        sceneRoot->transform->position = Vector3(0, 0, 0);
        
        // Create initial objects from pool и добавляем их в иерархию
        for (int i = 0; i < 8; i++) {
            GameObject* newObj = objectPool.GetObject();
            if (newObj) {
                newObj->SetParent(sceneRoot);
            }
        }
        
        std::cout << "Scene initialized with root object and " << objectPool.GetActiveCount() << " child objects" << std::endl;
    }
    
    std::vector<GameObject*> getObjects() {
        return getAllObjectsInHierarchy();
    }
    
    std::vector<GameObject*> getAllObjectsInHierarchy() {
        std::vector<GameObject*> allObjects;
        std::stack<GameObject*> stack;
        
        // Начинаем с корня сцены
        stack.push(sceneRoot);
        
        // Обходим иерархию в глубину
        while (!stack.empty()) {
            GameObject* current = stack.top();
            stack.pop();
            
            allObjects.push_back(current);
            
            // Добавляем детей
            auto children = current->GetChildren();
            for (auto child : children) {
                stack.push(child);
            }
        }
        
        return allObjects;
    }
    
    GameObject* createGameObject() {
        GameObject* newObj = objectPool.GetObject();
        if (newObj) {
            newObj->SetParent(sceneRoot);
        }
        return newObj;
    }
    
    void returnObject(GameObject* obj) {
        obj->SetParent(nullptr);
        objectPool.ReturnObject(obj);
    }
    
    void returnAllObjects() {
        // Убираем всех детей из иерархии перед возвратом в пул
        auto children = sceneRoot->GetChildren();
        for (auto child : children) {
            child->SetParent(nullptr);
        }
        objectPool.ReturnAllObjects();
    }
    
    void registerPhysicsObjects(PhysicsSystem& physics) {
        auto objects = getObjects();
        for (auto obj : objects) {
            if (obj->hasCollider && obj->hasRigidbody) {
                physics.RegisterPhysicsObject(obj);
            }
        }
    }
    
    void renderOptimized(RenderSystem& renderer, const Camera& camera) {
        auto objects = getObjects();
        for (auto obj : objects) {
            MeshRenderer* mesh = obj->getComponent<MeshRenderer>();
            if (mesh) {
                renderer.addToRenderQueue(obj->transform, mesh, camera);
            }
        }
        renderer.renderAllFromQueue(camera);
        
        // Рендерим связи иерархии
        renderer.renderHierarchyConnections(getAllObjectsInHierarchy(), camera);
    }
    
    GameObject* selectObject(const Vector2& screenPos, const Camera& camera, RenderSystem& renderer) {
    float closestDistance = std::numeric_limits<float>::max();
    GameObject* closestObject = nullptr;
    Vector3 hitPoint;

    auto objects = getObjects();
    for (auto obj : objects) {
        MeshRenderer* mesh = obj->getComponent<MeshRenderer>();
        if (mesh) {
            Vector3 testHit;
            if (renderer.raycast(screenPos, obj->transform, mesh, camera, testHit)) {
                float distance = (testHit - camera.position).length();
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestObject = obj;
                    hitPoint = testHit;
                }
            }
        }
    }

    // Выделяем найденный объект (или nullptr — снимет выделение)
    selectObject(closestObject);

    return selectedObject;
}
    
    void moveSelectedObject(const Vector2& screenDelta, const Camera& camera) {
        if (!selectedObject) return;
        
        Vector3 right = camera.getRight();
        Vector3 up = Vector3(0, 1, 0);
        
        float sensitivity = 0.01f;
        Vector3 moveDelta = right * screenDelta.x * sensitivity + up * screenDelta.y * sensitivity;
        
        selectedObject->transform->position = selectedObject->transform->position + moveDelta;
        
        Rigidbody* rb = selectedObject->getComponent<Rigidbody>();
        if (rb) {
            rb->velocity = Vector3(0, 0, 0);
            rb->angularVelocity = Vector3(0, 0, 0);
        }
    }
    
    void deselectObject() {
        if (selectedObject) {
            ObjectSelector* selector = selectedObject->getComponent<ObjectSelector>();
            if (selector) {
                selector->Deselect();
            }
            selectedObject = nullptr;
        }
    }
    
    GameObject* getSelectedObject() const { return selectedObject; }
    
    void update() {
        auto objects = getObjects();
        for (auto obj : objects) {
            obj->update();
        }
    }
    void selectObject(GameObject* obj) {
    if (selectedObject == obj) return;

    // Снять выделение с предыдущего объекта
    if (selectedObject) {
        ObjectSelector* selector = selectedObject->getComponent<ObjectSelector>();
        if (selector) selector->Deselect();
    }

    selectedObject = obj;

    // Выделить новый объект
    if (selectedObject) {
        ObjectSelector* selector = selectedObject->getComponent<ObjectSelector>();
        if (selector) selector->Select();
    }
}
    
    void clear() {
        selectedObject = nullptr;
        returnAllObjects();
    }
    
    int getObjectCount() const { 
        return objectPool.GetActiveCount() + 1; // +1 для корня сцены
    }
    
    int getPoolSize() const { return objectPool.GetPoolSize(); }
    int getAvailableObjects() const { return objectPool.GetAvailableCount(); }
    
    // Новые методы для работы с иерархией
    void setParent(GameObject* child, GameObject* parent) {
        if (child && parent) {
            child->SetParent(parent);
        }
    }
    
    void removeParent(GameObject* obj) {
        if (obj) {
            obj->SetParent(sceneRoot); // Возвращаем к корню сцены
        }
    }
    
    GameObject* getSceneRoot() const { return sceneRoot; }
};

// ==================== JOYSTICK & UI ====================
class Joystick {
private:
    int centerX, centerY;
    int handleX, handleY;
    int radius;
    bool active;
    
public:
    Joystick(int x, int y, int r) : centerX(x), centerY(y), handleX(x), handleY(y), radius(r), active(false) {}
    
    void setPosition(int x, int y) {
        int dx = x - centerX;
        int dy = y - centerY;
        float distance = sqrt(dx*dx + dy*dy);
        
        if (distance > radius) {
            float scale = radius / distance;
            dx = (int)(dx * scale);
            dy = (int)(dy * scale);
        }
        
        handleX = centerX + dx;
        handleY = centerY + dy;
        active = true;
    }
    
    void reset() {
        handleX = centerX;
        handleY = centerY;
        active = false;
    }
    
    Vector2 getInput() const {
        if (!active) return Vector2(0, 0);
        float x = (handleX - centerX) / (float)radius;
        float y = (handleY - centerY) / (float)radius;
        return Vector2(x, -y);
    }
    
    void draw(RenderSystem& renderer, Color baseColor, Color handleColor) {
        for (int angle = 0; angle < 360; angle++) {
            float rad = angle * 3.14159f / 180.0f;
            for (int r = radius - 5; r <= radius; r++) {
                int px = centerX + cos(rad) * r;
                int py = centerY + sin(rad) * r;
                if (px >= 0 && px < SCREEN_WIDTH && py >= 0 && py < SCREEN_HEIGHT) {
                    renderer.setPixelUI(px, py, baseColor);
                }
            }
        }
        
        int handleRadius = 15;
        for (int x = -handleRadius; x <= handleRadius; x++) {
            for (int y = -handleRadius; y <= handleRadius; y++) {
                if (x*x + y*y <= handleRadius*handleRadius) {
                    int px = handleX + x;
                    int py = handleY + y;
                    if (px >= 0 && px < SCREEN_WIDTH && py >= 0 && py < SCREEN_HEIGHT) {
                        renderer.setPixelUI(px, py, handleColor);
                    }
                }
            }
        }
    }
    
    bool isActive() const { return active; }
    int getCenterX() const { return centerX; }
    int getCenterY() const { return centerY; }
    int getRadius() const { return radius; }
};

class Button {
public:
    int x, y, width, height;
    std::string text;
    Color color;
    
    Button(int x, int y, int w, int h, std::string t, Color c = Color(100, 100, 255, 200))
        : x(x), y(y), width(w), height(h), text(t), color(c) {}
    
    bool isPressed(int touchX, int touchY) {
        return touchX >= x && touchX <= x + width && 
               touchY >= y && touchY <= y + height;
    }
    
    void draw(RenderSystem& renderer) {
        renderer.drawRect(x, y, width, height, Color(0, 0, 0, 200));
        renderer.drawRect(x + 1, y + 1, width - 2, height - 2, color);
        // Draw text in the center of button
        int textX = x + (width - text.length() * 10) / 2;
        int textY = y + (height - 20) / 2;
        renderer.drawText(text, textX, textY, Color(255, 255, 255), true);
    }
};

// ==================== MAIN ====================
int main() {
    srand(time(0));
    
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return -1;
    }
    
    SDL_Window* window = SDL_CreateWindow(
        "Physics Engine with Object Hierarchy",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN
    );
    
    if (!window) {
        SDL_Quit();
        return -1;
    }
    
    RenderSystem renderer(window);
    Scene scene;
    Camera camera;
    PhysicsSystem physics;
    
    scene.Initialize();
    
    Joystick moveJoystick(100, SCREEN_HEIGHT - 100, 50);
    Joystick lookJoystick(SCREEN_WIDTH - 100, SCREEN_HEIGHT - 100, 50);
    
    Button addButton(SCREEN_WIDTH - 160, 10, 150, 40, "ADD CUBE", Color(100, 255, 100, 200));
    Button removeButton(SCREEN_WIDTH - 160, 60, 150, 40, "REMOVE", Color(255, 100, 100, 200));
    Button clearButton(SCREEN_WIDTH - 160, 110, 150, 40, "CLEAR ALL", Color(255, 100, 100, 200));
    Button physicsButton(SCREEN_WIDTH - 160, 160, 150, 40, "PHYSICS ON", Color(100, 100, 255, 200));
    Button hierarchyButton(10, SCREEN_HEIGHT - 50, 150, 40, "SHOW HIERARCHY", Color(200, 100, 200, 200));
    
    bool physicsEnabled = false;
    bool objectMoving = false;
    bool showHierarchy = true;
    int lastTouchX = 0, lastTouchY = 0;
    
    GameObject* lightObj = new GameObject("DirectionalLight");
    Light* light = lightObj->addComponent(std::make_unique<Light>());
    light->type = Light::DIRECTIONAL;
    light->color = Color(255, 255, 255);
    light->intensity = 1.0f;
    light->direction = Vector3(0.5f, -1.0f, 0.5f).normalized();
    renderer.AddLight(light);
    lightObj->SetParent(scene.getSceneRoot());
    
    scene.registerPhysicsObjects(physics);
    
    bool running = true;
    SDL_Event event;
    
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            
            if (event.type == SDL_FINGERDOWN || event.type == SDL_MOUSEBUTTONDOWN) {
                int touchX, touchY;
                if (event.type == SDL_FINGERDOWN) {
                    touchX = event.tfinger.x * SCREEN_WIDTH;
                    touchY = event.tfinger.y * SCREEN_HEIGHT;
                } else {
                    touchX = event.button.x;
                    touchY = event.button.y;
                }
                
                lastTouchX = touchX;
                lastTouchY = touchY;
                
                bool uiHandled = false;
                
                // Проверка кнопок
                if (hierarchyButton.isPressed(touchX, touchY)) {
                    showHierarchy = !showHierarchy;
                    hierarchyButton.text = showHierarchy ? "HIDE HIERARCHY" : "SHOW HIERARCHY";
                    uiHandled = true;
                }
                else if (addButton.isPressed(touchX, touchY)) {
                    GameObject* newCube = scene.createGameObject();
                    if (newCube) physics.RegisterPhysicsObject(newCube);
                    uiHandled = true;
                }
                else if (removeButton.isPressed(touchX, touchY)) {
                    GameObject* selected = scene.getSelectedObject();
                    if (selected) { scene.returnObject(selected); scene.deselectObject(); }
                    uiHandled = true;
                }
                else if (clearButton.isPressed(touchX, touchY)) {
                    scene.clear();
                    physics.Clear();
                    uiHandled = true;
                }
                else if (physicsButton.isPressed(touchX, touchY)) {
                    physicsEnabled = !physicsEnabled;
                    physicsButton.text = physicsEnabled ? "PHYSICS OFF" : "PHYSICS ON";
                    uiHandled = true;
                }
                
                if (!uiHandled) {
                    // Проверка джойстиков
                    int dx1 = touchX - moveJoystick.getCenterX();
                    int dy1 = touchY - moveJoystick.getCenterY();
                    if (dx1*dx1 + dy1*dy1 <= moveJoystick.getRadius() * moveJoystick.getRadius()) {
                        moveJoystick.setPosition(touchX, touchY);
                        uiHandled = true;
                    }
                    
                    int dx2 = touchX - lookJoystick.getCenterX();
                    int dy2 = touchY - lookJoystick.getCenterY();
                    if (!uiHandled && dx2*dx2 + dy2*dy2 <= lookJoystick.getRadius() * lookJoystick.getRadius()) {
                        lookJoystick.setPosition(touchX, touchY);
                        uiHandled = true;
                    }
                }
                
                if (!uiHandled && showHierarchy && touchX >= 10 && touchX <= 260 && touchY >= 200 && touchY <= SCREEN_HEIGHT - 60) {
    uiHandled = true;
    auto allObjects = scene.getAllObjectsInHierarchy();
    int yPos = 230;
    bool hit = false;
    for (auto obj : allObjects) {
        if (yPos > SCREEN_HEIGHT - 60) break;
        if (touchY >= yPos && touchY < yPos + 20) {
            scene.selectObject(obj);
            hit = true;
            break;
        }
        yPos += 20;
    }
    if (!hit) {
        // Клик в пустую область панели — снимаем выделение
        scene.selectObject(nullptr);
    }
}
                
                if (!uiHandled) {
                    // Попытка выделить объект в сцене
                    GameObject* selected = scene.selectObject(Vector2(touchX, touchY), camera, renderer);
                    if (selected) {
                        objectMoving = true;
                    }
                }
            }
            
            if (event.type == SDL_FINGERUP || event.type == SDL_MOUSEBUTTONUP) {
                moveJoystick.reset();
                lookJoystick.reset();
                objectMoving = false;
            }
            
            if (event.type == SDL_FINGERMOTION || event.type == SDL_MOUSEMOTION) {
                int touchX, touchY;
                if (event.type == SDL_FINGERMOTION) {
                    touchX = event.tfinger.x * SCREEN_WIDTH;
                    touchY = event.tfinger.y * SCREEN_HEIGHT;
                } else {
                    touchX = event.motion.x;
                    touchY = event.motion.y;
                }
                
                if (moveJoystick.isActive()) {
                    moveJoystick.setPosition(touchX, touchY);
                }
                if (lookJoystick.isActive()) {
                    lookJoystick.setPosition(touchX, touchY);
                }
                
                if (objectMoving && scene.getSelectedObject()) {
                    int deltaX = touchX - lastTouchX;
                    int deltaY = touchY - lastTouchY;
                    scene.moveSelectedObject(Vector2(deltaX, deltaY), camera);
                }
                
                lastTouchX = touchX;
                lastTouchY = touchY;
            }
        }
        
        // Движение камеры
        Vector2 moveInput = moveJoystick.getInput();
        Vector2 lookInput = lookJoystick.getInput();
        
        if (fabs(moveInput.x) > 0.1f || fabs(moveInput.y) > 0.1f) {
            Vector3 forward = camera.getForward();
            Vector3 right = camera.getRight();
            Vector3 moveDirection = right * moveInput.x + forward * moveInput.y;
            camera.move(moveDirection);
        }
        
        if (fabs(lookInput.x) > 0.1f || fabs(lookInput.y) > 0.1f) {
            camera.look(lookInput.x, -lookInput.y);
        }
        
        // Физика
        if (physicsEnabled && !objectMoving) {
            auto objects = scene.getObjects();
            physics.Update(objects, 0.016f);
        }
        
        scene.update();
        
        // Рендеринг
        renderer.clear(Color(50, 50, 50));
        scene.renderOptimized(renderer, camera);
        
        moveJoystick.draw(renderer, Color(100, 100, 100, 150), Color(200, 200, 200));
        lookJoystick.draw(renderer, Color(100, 100, 100, 150), Color(200, 200, 200));
        
        addButton.draw(renderer);
        removeButton.draw(renderer);
        clearButton.draw(renderer);
        physicsButton.draw(renderer);
        hierarchyButton.draw(renderer);
        
        // Статусы
        if (physicsEnabled) {
            renderer.drawRect(10, 10, 120, 25, Color(0, 255, 0, 200));
            renderer.drawText("PHYSICS: ON", 15, 12, Color(255, 255, 255), true);
        } else {
            renderer.drawRect(10, 10, 120, 25, Color(255, 0, 0, 200));
            renderer.drawText("PHYSICS: OFF", 15, 12, Color(255, 255, 255), true);
        }
        
        renderer.drawRect(10, 45, 250, 80, Color(0, 0, 0, 150));
        renderer.drawText("Objects: " + std::to_string(scene.getObjectCount()) + "/" + std::to_string(scene.getPoolSize()), 15, 50, Color(255, 255, 255), true);
        renderer.drawText("Available: " + std::to_string(scene.getAvailableObjects()), 15, 75, Color(255, 255, 255), true);
        renderer.drawText("FPS: 60", 15, 100, Color(255, 255, 255), true);
        
        GameObject* selected = scene.getSelectedObject();
        if (selected) {
            renderer.drawRect(10, 135, 250, 40, Color(50, 50, 200, 200));
            renderer.drawText("SELECTED: " + selected->name, 15, 137, Color(255, 255, 255), true);
            
            if (selected->GetParent()) {
                renderer.drawText("Parent: " + selected->GetParent()->name, 15, 157, Color(255, 255, 255), true);
            } else {
                renderer.drawText("Parent: None", 15, 157, Color(255, 255, 255), true);
            }
        }
        
        // Панель иерархии
        if (showHierarchy) {
            int panelWidth = 250;
            renderer.drawRect(10, 200, panelWidth, SCREEN_HEIGHT - 250, Color(0, 0, 0, 180));
            renderer.drawText("SCENE HIERARCHY", 15, 205, Color(255, 255, 255), true);
            
            int yPos = 230;
            auto allObjects = scene.getAllObjectsInHierarchy();
            for (auto obj : allObjects) {
                if (yPos > SCREEN_HEIGHT - 60) break;
                
                std::string displayName = obj->name;
                if (obj == selected) {
                    displayName = "> " + displayName + " <";
                }
                
                int level = 0;
                GameObject* current = obj;
                while (current->GetParent() && current->GetParent() != scene.getSceneRoot()) {
                    level++;
                    current = current->GetParent();
                }
                std::string indent(level * 3, ' ');
                std::string prefix = (obj->GetChildren().empty()) ? "• " : "+ ";
                
                if (displayName.length() > 20) {
                    displayName = displayName.substr(0, 17) + "...";
                }
                
                renderer.drawText(indent + prefix + displayName, 20, yPos,
                                (obj == selected) ? Color(255, 255, 0) : Color(255, 255, 255), true);
                yPos += 20;
            }
            
            renderer.drawRect(10, SCREEN_HEIGHT - 45, panelWidth, 40, Color(30, 30, 60, 200));
            renderer.drawText("Scene Root: " + scene.getSceneRoot()->name, 15, SCREEN_HEIGHT - 40, Color(200, 200, 255), true);
            renderer.drawText("Children: " + std::to_string(scene.getSceneRoot()->GetChildren().size()), 15, SCREEN_HEIGHT - 20, Color(200, 200, 255), true);
        }
        
        renderer.present();
        SDL_Delay(16);
    }
    
    delete lightObj;
    SDL_Quit();
    return 0;
}