//
// Created by levyvonet on 29/12/2021.
//

#include "visualization.h"

#define OUTLINE_THICKNESS (0.5f)
#define WINDOW_BUFFER_FACTOR (0.95)

vector<sf::Color> AGENT_COLORS{
        sf::Color::Red,
        sf::Color::Green,
        sf::Color::Blue,
        sf::Color::Yellow,
        sf::Color::Magenta,
        sf::Color::Cyan,
};

TileDrawable::TileDrawable(float tile_size, Location l) : tile_size(tile_size) {
    this->position = sf::Vector2i(l.col * tile_size, l.row * tile_size);
    this->tile_image = sf::RectangleShape(sf::Vector2f(this->tile_size, this->tile_size));
    this->tile_image.setPosition(sf::Vector2f(this->position));
    this->tile_image.setOutlineColor(sf::Color::Black);
    this->tile_image.setOutlineThickness(OUTLINE_THICKNESS);
    this->tile_image.setFillColor(sf::Color::White);
}

void TileDrawable::draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const {
    renderTarget.draw(this->tile_image);
}


AgentDrawable::AgentDrawable(float tile_size, Location l, sf::Color color) :
        tile_size(tile_size), color(color) {
    this->position = sf::Vector2i(l.col * tile_size, l.row * tile_size);

    this->agent_image = sf::CircleShape(tile_size / 2);
    this->agent_image.setPosition(sf::Vector2f(this->position));
    this->agent_image.setOutlineColor(sf::Color::Black);
    this->agent_image.setOutlineThickness(OUTLINE_THICKNESS);
    this->agent_image.setFillColor(this->color);
}

void AgentDrawable::draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const {
    renderTarget.draw(this->agent_image);
}

GoalDrawable::GoalDrawable(float tile_size, Location l, sf::Color color) :
        tile_size(tile_size), color(color) {
    this->position = sf::Vector2i(l.col * tile_size, l.row * tile_size);

    this->goal_image = sf::CircleShape(tile_size / 2, 3);
    this->goal_image.setPosition(sf::Vector2f(this->position));
    this->goal_image.setOutlineColor(sf::Color::Black);
    this->goal_image.setOutlineThickness(OUTLINE_THICKNESS);
    this->goal_image.setFillColor(this->color);
}

void GoalDrawable::draw(sf::RenderTarget &renderTarget, sf::RenderStates renderStates) const {
    renderTarget.draw(this->goal_image);
}


GridDrawable::GridDrawable(int row_count, int col_count, int tile_size) :
        row_count(row_count), col_count(col_count), tile_size(tile_size) {}

void GridDrawable::draw(sf::RenderTarget &render_target, sf::RenderStates render_states) const {
    vector<TileDrawable> tiles;


    /* Draw the blank tiles */
    for (int i = 0; i < this->row_count; ++i) {
        for (int j = 0; j < this->col_count; ++j) {
            TileDrawable t = TileDrawable(this->tile_size,
                                          Location(i, j, 0));
            render_target.draw(t);
        }
    }

    /* Draw the agents locations on the grid */

}


MapfEnvDrawable::MapfEnvDrawable(MapfEnv *env) : env(env),
                                                 grid_drawable(env->grid->max_row + 1,
                                                               env->grid->max_col + 1,
                                                               min(sf::VideoMode::getDesktopMode().height *
                                                                   WINDOW_BUFFER_FACTOR / (env->grid->max_row + 1),
                                                                   sf::VideoMode::getDesktopMode().width *
                                                                   WINDOW_BUFFER_FACTOR / (env->grid->max_col + 1))) {
    int row_count = this->env->grid->max_row + 1;
    int col_count = this->env->grid->max_col + 1;

    float tile_height = sf::VideoMode::getDesktopMode().height * WINDOW_BUFFER_FACTOR / row_count;
    float tile_width = sf::VideoMode::getDesktopMode().width * WINDOW_BUFFER_FACTOR / col_count;

    float tile_size = min(tile_height, tile_width);

    this->grid_drawable = GridDrawable(row_count, col_count, tile_size);
}

void MapfEnvDrawable::draw(sf::RenderTarget &render_target, sf::RenderStates render_states) const {

    /* Draw the grid */
    this->grid_drawable.draw(render_target, render_states);

    /* Draw the agents */
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        AgentDrawable agent = AgentDrawable(this->grid_drawable.tile_size,
                                            this->env->s->locations[i],
                                            AGENT_COLORS[i]);
        GoalDrawable goal = GoalDrawable(this->grid_drawable.tile_size,
                                         this->env->goal_state->locations[i],
                                         AGENT_COLORS[i]);

        agent.draw(render_target, render_states);
        goal.draw(render_target, render_states);
    }
}


void render(MapfEnv *env, sf::RenderWindow *window) {
    MapfEnvDrawable env_drawable = MapfEnvDrawable(env);
    window->draw(env_drawable);
}