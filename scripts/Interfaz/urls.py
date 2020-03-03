from django.urls import path
from django.urls import include, re_path
from django.views.generic.base import RedirectView
from . import views

favicon_view = RedirectView.as_view(url='/static/favicon.png', permanent=True)

urlpatterns = [
    path('', views.index, name='index'),
    path('favicon.ico', favicon_view, name='favicon'),
	path('roboticArm', views.roboticArm, name='roboticArm'),
	path('auto', views.auto, name='auto'),
	path('sensors', views.sensors, name='sensors'),
    path('status', views.status, name='status'),

]