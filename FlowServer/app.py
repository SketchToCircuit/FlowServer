from re import A
import falcon.asgi

from .enpoints import Neural

def create_app():
    neural = Neural()
    app = falcon.asgi.App()
    app.add_route('/', neural)
    return app