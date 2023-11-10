#include <SFML/Graphics.hpp>
#include <box2d/box2d.h>

const float M2P = 20.0f;
const float P2M = 1/20.0f;

class Figure{
public:
    Figure(b2World &world, sf::Vector2f pos, sf::Color color){}

    virtual void Draw(sf::RenderWindow &win){}
    virtual void Update(){}

    b2Body* GetBody() {return body;}

protected:
    b2Body* body{};
    sf::RectangleShape rectangleShape;

    void DefBody(b2World &world, sf::Vector2f pos, float angle, b2BodyType type){
        b2BodyDef bodyDef;
        bodyDef.type = type;
        bodyDef.position.Set(pos.x * P2M, pos.y * P2M);
        bodyDef.angle = angle;
        body = world.CreateBody(&bodyDef);
    }

    void Rectangle(sf::Vector2f size, float density, float friction, sf::Color color){
        rectangleShape.setSize(size);
        rectangleShape.setPosition(body->GetPosition().x * M2P, body->GetPosition().y * M2P);
        rectangleShape.setRotation(body->GetAngle() * 180 / b2_pi);
        rectangleShape.setOrigin(size.x/2, size.y/2);
        rectangleShape.setFillColor(color);

        b2PolygonShape shape;
        shape.SetAsBox((size.x / 2) * P2M, (size.y / 2) * P2M);

        Fixture(density, friction, shape);
    }

    void Fixture(float density, float friction, b2Shape &shape){
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = density;
        fixtureDef.friction = friction;
        body->CreateFixture(&fixtureDef);
    }
};

//Cubos

class dynamicCube : public Figure {
public:
    dynamicCube(b2World &world, sf::Vector2f pos, sf::Vector2f size, float angle, float density, float friction, sf::Color color = sf::Color::Blue) : Figure(world, pos, color){
        DefBody(world, pos, angle, b2_dynamicBody);
        Rectangle(size, density, friction, color);
    }

    void Draw(sf::RenderWindow &win) override{
        win.draw(rectangleShape);
    }

    void Update() override{
        sf::Vector2f position(body->GetPosition().x * M2P, body->GetPosition().y * M2P);
        float rotation = body->GetAngle() * 180 / b2_pi;
        rectangleShape.setPosition(position);
        rectangleShape.setRotation(rotation);
    }
};

class staticCube : public Figure{
public:
    staticCube(b2World &world, sf::Vector2f pos, sf::Vector2f size, float angle, float density, float friction, sf::Color color = sf::Color::Red) : Figure(world, pos, color){
        DefBody(world, pos, angle, b2_staticBody);
        Rectangle(size, density, friction, color);
    }

    void Draw(sf::RenderWindow &win) override{
        win.draw(rectangleShape);
    }

    void Update() override{
        sf::Vector2f position(body->GetPosition().x * M2P, body->GetPosition().y * M2P);
        float rotation = body->GetAngle() * 180 / b2_pi;
        rectangleShape.setPosition(position);
        rectangleShape.setRotation(rotation);
    }
};
void Revolute(b2World &world, b2Body* pivotBody, b2Body* bodyA){
    b2RevoluteJointDef jointDef;
    jointDef.Initialize(pivotBody, bodyA, pivotBody->GetWorldCenter());
    jointDef.enableLimit = false;

    world.CreateJoint(&jointDef);
}

b2Joint* Prismatic(b2World &world, b2Body* pivotBody, b2Body* bodyA, b2Vec2 worldAxis, float upper, float lower){
    b2PrismaticJointDef jointDef;
    jointDef.Initialize(pivotBody, bodyA, pivotBody->GetWorldCenter(), worldAxis);
    jointDef.enableLimit = true;
    jointDef.upperTranslation = upper;
    jointDef.lowerTranslation = lower;

    b2Joint* joint = world.CreateJoint(&jointDef);
    return joint;
}

void Gear(b2World &world, b2Body* bodyA, b2Body* bodyB, b2Joint* jointA, b2Joint* jointB){
    b2GearJointDef jointDef;
    jointDef.bodyA = bodyA;
    jointDef.bodyB = bodyB;
    jointDef.joint1 = jointA;
    jointDef.joint2 = jointB;

    world.CreateJoint(&jointDef);
}

void Pulley(b2World &world, b2Body* bodyA, b2Body* bodyB, b2Vec2 p1, b2Vec2 p2){
    b2Vec2 anchor1 = bodyA->GetWorldCenter();
    b2Vec2 anchor2 = bodyB->GetWorldCenter();

    b2Vec2 groundAnchor1(p1.x, p1.y);
    b2Vec2 groundAnchor2(p2.x, p2.y + 10.0f);

    float ratio = 1.0f;

    b2PulleyJointDef jointDef;
    jointDef.Initialize(bodyA, bodyB, groundAnchor1, groundAnchor2, anchor1, anchor2, ratio);

    world.CreateJoint(&jointDef);
}
int main() {
    sf::RenderWindow window(sf::VideoMode(1270, 720), "Prueba");

    b2Vec2 gravity(0.0f, 0.05f);
    b2World world(gravity);

#pragma region BODIES
    sf::Vector2f sizeStandar(20.0f, 20.0f);
        //Dynamics
    dynamicCube Character(world, sf::Vector2f(100.0f, 10.0f), sizeStandar, 0.0f, 50.0f, 0.0, sf::Color::Yellow);
    dynamicCube Character2(world, sf::Vector2f(850, 350), sizeStandar, 0.0f, 0.1f, 0.3, sf::Color::Green);
    dynamicCube Character3(world, sf::Vector2f(1000, 500), sizeStandar, 0.0f, 0.1f, 0.3, sf::Color::Red);
    dynamicCube boxA(world, sf::Vector2f(850.0f, 250.0f), sf::Vector2f(20.0f, 60.0f), 0.0f, 1.5f, 0.3f, sf::Color::Red);
    dynamicCube BoxGearA(world, sf::Vector2f(1000.0f, 530.0f), sf::Vector2f(100.0f, 25.0f), 0.0f, 0.1f, 0.3f, sf::Color::Cyan);
    dynamicCube BoxGearB(world, sf::Vector2f(200.0f, 650.0f), sf::Vector2f(100.0f, 25.0f), 0.0f, 0.1f, 0.3f, sf::Color::Cyan);
    dynamicCube boxPulleyA(world, sf::Vector2f(465.0f, 400.0f), sizeStandar, 0.0f, 1.0f, 0.0f, sf::Color::Magenta);
    dynamicCube boxPulleyB(world, sf::Vector2f(500.0f, -30.0f), sizeStandar, 0.0f, 1.0f, 0.0f, sf::Color::Yellow);
    dynamicCube boxPrisA(world, sf::Vector2f(550.0f, 550.0f), sizeStandar, 0.0f, 0.1f, 0.0f, sf::Color::Blue);

    sf::Vector2f sizeBases(250.0f, 5.0f);
    sf::Color color = sf::Color::White;

        //Statics
    staticCube base1(world, sf::Vector2f(100.0f, 50.0f), sizeBases, 0.2f, 0.0f, 0.4f, color);
    staticCube base2(world, sf::Vector2f(900.0f, 400.0f), sizeBases, 0.0f, 0.0f, 0.0f, color);
    staticCube base3(world, sf::Vector2f(200.0f, 350.0f), sf::Vector2f(2.0f, 400.0f), 0.0f, 0.0f, 0.3f, color);
    staticCube base4(world, sf::Vector2f(150, 500), sf::Vector2f(100.0f, 5.0f), 0.0f, 0.0f, 0.3, color);
    staticCube pivot(world, sf::Vector2f(850.0f, 300.0f), sizeStandar, 0.0, 0.01f, 0.0f, color);
    staticCube pivotGear(world, sf::Vector2f(405.0f, 400.0f), sizeStandar, 0.0f, 0.01f, 0.0f, color);
    staticCube pivotPris(world, sf::Vector2f(680.0f, 600.0f), sizeStandar, 0.0f, 0.0f, 0.0f, color);
#pragma endregion

#pragma region JOINTS
    Revolute(world, pivot.GetBody(), boxA.GetBody());

    b2Joint* jointA = Prismatic(world, pivotGear.GetBody(), BoxGearA.GetBody(), b2Vec2(0.0f, 1.0f), 2.0f, -2.0f);
    b2Joint* jointB = Prismatic(world, pivotGear.GetBody(), BoxGearB.GetBody(), b2Vec2(0.0f, 1.0f), 2.0f, -2.0f);
    Gear(world, BoxGearA.GetBody(), BoxGearB.GetBody(), jointA, jointB);

    Pulley(world, boxPulleyA.GetBody(), boxPulleyB.GetBody(), b2Vec2(boxPulleyA.GetBody()->GetPosition().x, 50), b2Vec2(boxPulleyB.GetBody()->GetPosition().y, 50));

    Prismatic(world, pivotPris.GetBody(), boxPrisA.GetBody(), b2Vec2(1.0f, 0.0f), 10.0f, -10.0f);
#pragma endregion

    while(window.isOpen()){
        sf::Event event{};
        while(window.pollEvent(event)){
            if(event.type == sf::Event::Closed) window.close();
        }
        window.clear(sf::Color::Black);

        world.Step(1.0f / 60.0f, 8, 3);

#pragma region Draw&Update
        //Draw
        Character.Draw(window);
        boxA.Draw(window);
        BoxGearA.Draw(window);
        BoxGearB.Draw(window);
        Character2.Draw(window);
        boxPulleyA.Draw(window);
        Character3.Draw(window);
        boxPulleyB.Draw(window);
        boxPrisA.Draw(window);
        base1.Draw(window);
        base2.Draw(window);
        base3.Draw(window);
        base4.Draw(window);
        pivot.Draw(window);
        pivotGear.Draw(window);
        pivotPris.Draw(window);

        //Update
        Character.Update();
        boxA.Update();
        BoxGearA.Update();
        BoxGearB.Update();
        Character2.Update();
        boxPulleyA.Update();
        boxPulleyB.Update();
        Character3.Update();
        boxPrisA.Update();
#pragma endregion

        window.display();
    }

    return 0;
}