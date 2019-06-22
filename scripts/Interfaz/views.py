from django.shortcuts import render,redirect,render_to_response
from django.http import HttpResponse,StreamingHttpResponse
from django.http import Http404

# Create your views here.

def index(request):

	return render(request, 'index.html')

def roboticArm(request):

	return render(request, 'roboticArm.html')


def auto(request):

	return render(request, 'auto.html')


def geo(request):

	return render(request, 'geo.html')


def status(request):

	return render(request, 'status.html')