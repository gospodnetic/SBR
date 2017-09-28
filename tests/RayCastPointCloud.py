# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-09-27 15:19:29
# @Last Modified by:   petra
# @Last Modified time: 2017-09-28 10:17:17
#
# Script based on the official VTK example script for generating random spherical point cloud
# PointSource.py
#
# Generate spherical point cloud with points only on the surface and cast ray at it.
#
import vtk

# create a rendering window and renderer
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)

# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

# Generate sphere point cloud with points only on the surface
src = vtk.vtkPointSource()
src.SetCenter(0,0,0)
src.SetNumberOfPoints(350)
src.SetRadius(3)
src.SetDistributionToShell()
src.Update()

print (type(src))

# mapper
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(src.GetOutputPort())

# actor
actor = vtk.vtkActor()
actor.SetMapper(mapper)
actor.GetProperty().SetColor(1,0,0) # (R,G,B)

# assign actor to the renderer
ren.AddActor(actor)

# enable user interface interactor
renWin.Render()
iren.Initialize()
iren.Start()