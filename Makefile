PY=.venv/bin/python
.PHONY: all setup model config urdf analysis bom export render sim test clean
all: model config urdf analysis bom export render test

setup:
	python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

model:      ## CAD -> out/model.json (kinematics + mass model)
	cd cad && ../$(PY) export_model.py
config: model  ## firmware joint table from CAD
	$(PY) tools/gen_joint_config.py
urdf: model
	$(PY) sim/make_urdf.py
analysis: model
	$(PY) analysis/torque.py > out/torque_report.txt
	$(PY) analysis/strength.py > out/strength_report.txt
bom:
	$(PY) bom/build_bom.py
export:     ## STEP / STL / GLB + print plan
	$(PY) cad/export_all.py
render:
	$(PY) cad/render.py
sim:
	$(PY) sim/stand_test.py --crouch 15
	$(PY) sim/walk_test.py --steps 8 --step 40
test:
	$(PY) -m pytest -q tests
clean:
	rm -rf out/stl out/stl_print out/step out/*.glb out/*.step
