
// make build && make run

// g++ rasterizer.cpp -o rasterizer $(pkg-config --cflags --libs sdl2) && ./rasterizer test-cube.obj
// ./rasterizer test-cube.obj

#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>

struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

struct Triangle
{
    Vertex v0, v1, v2;
};

class OBJLoader
{
public:
    static std::vector<Triangle> loadOBJ(const std::string &filename)
    {
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;
        std::vector<Triangle> triangles;

        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v")
            {
                // Vertex position
                float x, y, z;
                iss >> x >> y >> z;
                positions.push_back(glm::vec3(x, y, z));
            }
            else if (prefix == "vn")
            {
                // Vertex normal
                float x, y, z;
                iss >> x >> y >> z;
                normals.push_back(glm::normalize(glm::vec3(x, y, z)));
            }
            else if (prefix == "f")
            {
                // Face
                std::string v1, v2, v3;
                iss >> v1 >> v2 >> v3;

                Triangle tri;
                parseVertex(v1, positions, normals, tri.v0);
                parseVertex(v2, positions, normals, tri.v1);
                parseVertex(v3, positions, normals, tri.v2);

                triangles.push_back(tri);
            }
        }

        return triangles;
    }

private:
    static void parseVertex(const std::string &vertexStr,
                            const std::vector<glm::vec3> &positions,
                            const std::vector<glm::vec3> &normals,
                            Vertex &vertex)
    {
        std::istringstream iss(vertexStr);
        std::string indexStr;
        std::vector<int> indices;

        while (std::getline(iss, indexStr, '/'))
        {
            if (!indexStr.empty())
            {
                indices.push_back(std::stoi(indexStr) - 1); // OBJ indices are 1-based
            }
            else
            {
                indices.push_back(-1);
            }
        }

        // Position (always present)
        vertex.position = positions[indices[0]];

        // Normal (if present)
        if (indices.size() >= 3 && indices[2] != -1)
        {
            vertex.normal = normals[indices[2]];
        }
        else
        {
            vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f); // Default normal
        }
    }
};

class Rasterizer
{
private:
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *frameBuffer;
    uint32_t *pixels;
    int width, height;
    std::vector<float> depthBuffer;
    std::vector<Triangle> mesh;

    // MVP matrices
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;

    // Lighting parameters
    glm::vec3 lightDir;
    glm::vec3 lightColor;
    float ambientStrength;

public:
    Rasterizer(int w, int h) : width(w), height(h)
    {
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Software Rasterizer",
                                  SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                  width, height, SDL_WINDOW_SHOWN);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        frameBuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                        SDL_TEXTUREACCESS_STREAMING, width, height);

        pixels = new uint32_t[width * height];
        depthBuffer.resize(width * height, 1.0f);

        // Initialize matrices
        model = glm::mat4(1.0f);
        view = glm::lookAt(
            glm::vec3(0.0f, 0.0f, 3.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f));
        projection = glm::perspective(glm::radians(45.0f),
                                      (float)width / (float)height, 0.1f, 100.0f);

        // Initialize lighting
        lightDir = glm::normalize(glm::vec3(-1.0f, -1.0f, -1.0f));
        lightColor = glm::vec3(1.0f, 1.0f, 1.0f);
        ambientStrength = 0.1f;
    }

    ~Rasterizer()
    {
        delete[] pixels;
        SDL_DestroyTexture(frameBuffer);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    void loadMesh(const std::string &filename)
    {
        mesh = OBJLoader::loadOBJ(filename);
    }

    void clear()
    {
        std::fill(pixels, pixels + width * height, 0);
        std::fill(depthBuffer.begin(), depthBuffer.end(), 1.0f);
    }

    void setPixel(int x, int y, const glm::vec3 &color)
    {
        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            // Convert float RGB [0,1] to uint32_t ARGB
            uint32_t r = static_cast<uint32_t>(std::min(255.0f, color.r * 255.0f));
            uint32_t g = static_cast<uint32_t>(std::min(255.0f, color.g * 255.0f));
            uint32_t b = static_cast<uint32_t>(std::min(255.0f, color.b * 255.0f));
            pixels[y * width + x] = (0xFF << 24) | (r << 16) | (g << 8) | b;
        }
    }

    glm::vec3 barycentric(const glm::vec2 &p, const glm::vec2 &a,
                          const glm::vec2 &b, const glm::vec2 &c)
    {
        glm::vec2 v0 = b - a, v1 = c - a, v2 = p - a;
        float d00 = glm::dot(v0, v0);
        float d01 = glm::dot(v0, v1);
        float d11 = glm::dot(v1, v1);
        float d20 = glm::dot(v2, v0);
        float d21 = glm::dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;
        return glm::vec3(u, v, w);
    }

    void drawTriangle(const Triangle &triangle)
    {
        // Transform vertices and normals
        glm::mat4 mvp = projection * view * model;
        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));

        glm::vec4 clip0 = mvp * glm::vec4(triangle.v0.position, 1.0f);
        glm::vec4 clip1 = mvp * glm::vec4(triangle.v1.position, 1.0f);
        glm::vec4 clip2 = mvp * glm::vec4(triangle.v2.position, 1.0f);

        glm::vec3 worldNormal0 = normalMatrix * triangle.v0.normal;
        glm::vec3 worldNormal1 = normalMatrix * triangle.v1.normal;
        glm::vec3 worldNormal2 = normalMatrix * triangle.v2.normal;

        // Perspective divide
        glm::vec3 ndc0 = glm::vec3(clip0) / clip0.w;
        glm::vec3 ndc1 = glm::vec3(clip1) / clip1.w;
        glm::vec3 ndc2 = glm::vec3(clip2) / clip2.w;

        // Viewport transform
        glm::vec2 screen0 = glm::vec2((ndc0.x + 1.0f) * width / 2.0f,
                                      (1.0f - ndc0.y) * height / 2.0f);
        glm::vec2 screen1 = glm::vec2((ndc1.x + 1.0f) * width / 2.0f,
                                      (1.0f - ndc1.y) * height / 2.0f);
        glm::vec2 screen2 = glm::vec2((ndc2.x + 1.0f) * width / 2.0f,
                                      (1.0f - ndc2.y) * height / 2.0f);

        // Calculate bounding box
        int minX = std::max(0, (int)std::min({screen0.x, screen1.x, screen2.x}));
        int maxX = std::min(width - 1, (int)std::max({screen0.x, screen1.x, screen2.x}));
        int minY = std::max(0, (int)std::min({screen0.y, screen1.y, screen2.y}));
        int maxY = std::min(height - 1, (int)std::max({screen0.y, screen1.y, screen2.y}));

        // Rasterize
        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                glm::vec2 p(x + 0.5f, y + 0.5f);
                glm::vec3 bc = barycentric(p, screen0, screen1, screen2);

                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0)
                {
                    float depth = bc.x * ndc0.z + bc.y * ndc1.z + bc.z * ndc2.z;
                    int index = y * width + x;

                    if (depth < depthBuffer[index])
                    {
                        depthBuffer[index] = depth;

                        // Interpolate normal
                        glm::vec3 normal = glm::normalize(
                            bc.x * worldNormal0 +
                            bc.y * worldNormal1 +
                            bc.z * worldNormal2);

                        // Calculate lighting
                        float diffuse = std::max(0.0f, glm::dot(normal, -lightDir));
                        glm::vec3 color = lightColor * (ambientStrength + diffuse);

                        setPixel(x, y, color);
                    }
                }
            }
        }
    }

    void render()
    {
        clear();

        // Draw all triangles in the mesh
        for (const auto &triangle : mesh)
        {
            drawTriangle(triangle);
        }

        SDL_UpdateTexture(frameBuffer, NULL, pixels, width * sizeof(uint32_t));
        SDL_RenderCopy(renderer, frameBuffer, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    void setModelMatrix(const glm::mat4 &m) { model = m; }
    void setViewMatrix(const glm::mat4 &v) { view = v; }
    void setProjectionMatrix(const glm::mat4 &p) { projection = p; }
    void setLightDirection(const glm::vec3 &dir) { lightDir = glm::normalize(dir); }
    void setLightColor(const glm::vec3 &color) { lightColor = color; }
    void setAmbientStrength(float strength) { ambientStrength = strength; }
};

// Example usage
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_obj_file>" << std::endl;
        return 1;
    }

    Rasterizer rasterizer(800, 600);

    try
    {
        rasterizer.loadMesh(argv[1]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading mesh: " << e.what() << std::endl;
        return 1;
    }

    bool running = true;
    SDL_Event event;
    float rotation = 0.0f;

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        // Update rotation
        rotation += 0.1f;
        glm::mat4 model = glm::rotate(glm::mat4(1.0f), rotation,
                                      glm::vec3(0.0f, 1.0f, 0.0f));
        rasterizer.setModelMatrix(model);

        // Render frame
        rasterizer.render();

        SDL_Delay(16); // Cap at ~60 FPS
    }

    return 0;
}