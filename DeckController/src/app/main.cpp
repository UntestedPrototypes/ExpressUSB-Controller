#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>

#include "Backend.h"

int main(int argc, char* argv[]) {
    QGuiApplication app(argc, argv);
    app.setApplicationName("DeckController");
    app.setOrganizationName("PentaPod");
    QQuickStyle::setStyle("Basic");

    Backend backend;
    // Losing the focus (another window, Steam overlay, suspend) disarms the robot
    QObject::connect(&app, &QGuiApplication::applicationStateChanged, &backend, [&](Qt::ApplicationState s) {
        if (s != Qt::ApplicationActive) backend.appInactive();
    });

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("backend", &backend);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
    engine.loadFromModule("DeckApp", "Main");
    return app.exec();
}
