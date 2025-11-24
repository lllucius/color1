#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define TCS3530_ADDR 0x39

#define INTEGRATION_TIME_DEFAULT 24
#define GAIN_DEFAULT 1

#define REF_X 95.047
#define REF_Y 100.000
#define REF_Z 108.883

#define MAX_OCTREE_DEPTH 8

typedef struct {
    float X, Y, Z;
    float IR, Clear;
} tcs3530_color_t;

typedef struct {
    float L, a, b;
} lab_color_t;

typedef struct OctreeNode {
    lab_color_t color;
    const char* name;
    struct OctreeNode* children[8];
} OctreeNode;

// Function prototypes
void tcs3530_init();
tcs3530_color_t tcs3530_read_color();
void determine_optimal_gain_integration();
float calculate_color_temperature(tcs3530_color_t *color);
void apply_white_balance_correction(tcs3530_color_t *color);
void adjust_color_temperature(tcs3530_color_t *color);
lab_color_t xyz_to_lab(tcs3530_color_t *xyz);
const char* match_color(lab_color_t *color);
tcs3530_color_t convert_raw_to_float(uint16_t rawX, uint16_t rawY, uint16_t rawZ, uint16_t rawIR, uint16_t rawClear);
OctreeNode* build_color_octree();
const char* search_octree(OctreeNode* node, lab_color_t *color);
OctreeNode* create_octree_node(lab_color_t color, const char* name);
void insert_color(OctreeNode* root, lab_color_t color, const char* name, int depth);
const char* find_closest_color(OctreeNode* node, lab_color_t *color, float *best_distance);

void tcs3530_init() {
    ESP_LOGI("TCS3530", "Initializing sensor...");
}

OctreeNode* create_octree_node(lab_color_t color, const char* name) {
    OctreeNode* node = (OctreeNode*)malloc(sizeof(OctreeNode));
    node->color = color;
    node->name = name;
    memset(node->children, 0, sizeof(node->children));
    return node;
}

void insert_color(OctreeNode* root, lab_color_t color, const char* name, int depth) {
    if (depth >= MAX_OCTREE_DEPTH) {
        root->name = name;
        return;
    }
    int index = ((color.L > root->color.L) << 2) |
                ((color.a > root->color.a) << 1) |
                ((color.b > root->color.b));
    if (!root->children[index]) {
        root->children[index] = create_octree_node(color, name);
    }
    insert_color(root->children[index], color, name, depth + 1);
}

const char* find_closest_color(OctreeNode* node, lab_color_t *color, float *best_distance) {
    if (!node) return NULL;
    float distance = sqrtf(pow(color->L - node->color.L, 2) +
                            pow(color->a - node->color.a, 2) +
                            pow(color->b - node->color.b, 2));
    if (distance < *best_distance) {
        *best_distance = distance;
    }
    int index = ((color->L > node->color.L) << 2) |
                ((color->a > node->color.a) << 1) |
                ((color->b > node->color.b));
    const char* closest = node->name;
    for (int i = 0; i < 8; i++) {
        const char* child_result = find_closest_color(node->children[i], color, best_distance);
        if (child_result) closest = child_result;
    }
    return closest;
}

OctreeNode* build_color_octree() {
    OctreeNode* root = create_octree_node((lab_color_t){50, 0, 0}, "Neutral Gray");
    insert_color(root, (lab_color_t){75, 20, 50}, "Bright Red", 0);
    insert_color(root, (lab_color_t){30, -10, 20}, "Deep Blue", 0);
    insert_color(root, (lab_color_t){90, 0, 90}, "Vivid Magenta", 0);
    return root;
}

const char* search_octree(OctreeNode* node, lab_color_t *color) {
    float best_distance = 1e9;
    return find_closest_color(node, color, &best_distance);
}

tcs3530_color_t tcs3530_read_color() {
    tcs3530_color_t color = {0.5, 0.4, 0.3, 0.1, 1.0};
    return color;
}

const char* match_color(lab_color_t *color) {
    OctreeNode* root = build_color_octree();
    return search_octree(root, color);
}

void app_main() {
    tcs3530_init();
    determine_optimal_gain_integration();
    
    tcs3530_color_t color = tcs3530_read_color();
    apply_white_balance_correction(&color);
    adjust_color_temperature(&color);
    
    lab_color_t lab = xyz_to_lab(&color);
    const char* color_name = match_color(&lab);
    
    printf("Identified color: %s\n", color_name);
}


