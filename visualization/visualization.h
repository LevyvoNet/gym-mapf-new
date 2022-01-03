//
// Created by levyvonet on 29/12/2021.
//

#ifndef GYM_MAPF_VISUALIZATION_H
#define GYM_MAPF_VISUALIZATION_H

#include <SFML/Graphics.hpp>

#include <gym_mapf/gym_mapf.h>

using namespace std;

class TileDrawable : public sf::Drawable {
public:
    float tile_size;
    sf::RectangleShape tile_image;
    sf::Vector2i position;
    sf::Color fill_color;

    TileDrawable(float tile_size, Location l, sf::Color fill_color);

    void draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const override;
};


class AgentDrawable : public sf::Drawable {
public:
    float tile_size;
    sf::CircleShape agent_image;
    sf::Vector2i position;
    sf::Color color;

    AgentDrawable(float tile_size, Location l, sf::Color color);

    void draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const override;

};

class GoalDrawable : public sf::Drawable {
public:
    float tile_size;
    sf::CircleShape goal_image;
    sf::Vector2i position;
    sf::Color color;

    GoalDrawable(float tile_size, Location l, sf::Color color);

    void draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const override;
};

class GridDrawable : public sf::Drawable {
public:
    Grid* g;
    int tile_size;

    GridDrawable(Grid* g, int tile_size);

    void draw(sf::RenderTarget &render_target, sf::RenderStates render_states) const override;
};


class MapfEnvDrawable : public sf::Drawable {
public:
    MapfEnv *env;
    int tile_size;

    GridDrawable grid_drawable;

    MapfEnvDrawable(MapfEnv *env);

    void draw(sf::RenderTarget &render_target, sf::RenderStates render_states) const override;

};

void render(MapfEnv *env, sf::RenderWindow *window);

#endif //GYM_MAPF_VISUALIZATION_H
