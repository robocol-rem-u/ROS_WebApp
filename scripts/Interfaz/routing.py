from django.conf.urls import url
from django.urls import path

from . import consumers

websocket_urlpatterns = [
    path('ws/bgUpdate_traction/', consumers.bgUpdate_traction),
    path('ws/bgUpdate_status/', consumers.bgUpdate_status),
    path('ws/bgUpdate_autonomous/', consumers.bgUpdate_autonomous),
    path('ws/bgUpdate_roboticArm/', consumers.bgUpdate_roboticArm),
    path('ws/bgUpdate_geo/', consumers.bgUpdate_geo),
]