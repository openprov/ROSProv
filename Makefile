PROVCONVERT := provconvert
PYPROVCONVERT := prov-convert

# Generic rules to convert PROV-N to TTL or to SVG/PDF formats
%.json: %.provn
	$(PROVCONVERT) -infile $< -outfile $@	

%.ttl: %.provn
	$(PROVCONVERT) -infile $< -outfile $@

%.svg: %.json
	$(PYPROVCONVERT) -f svg $< $@

%.pdf: %.json
	$(PYPROVCONVERT) -f pdf $< $@	

%.png: %.json
	$(PYPROVCONVERT) -f png $< $@


TEMPLATES := templates

TEMPLATE_SRCFILES := $(wildcard $(TEMPLATES)/*.provn)
TEMPLATE_SVGS := $(patsubst %.provn, %.svg, ${TEMPLATE_SRCFILES})
TEMPLATE_PNGS := $(patsubst %.provn, %.png, ${TEMPLATE_SRCFILES})
TEMPLATE_PDFS := $(patsubst %.provn, %.pdf, ${TEMPLATE_SRCFILES})

templates: ${TEMPLATE_SVGS} ${TEMPLATE_PNGS} ${TEMPLATE_PDFS}


compile-templates:
	$(PROVCONVERT) -templatebuilder templates/config.json
	cd target/libs/templates; mvn clean install

example-expansion:
	target/bin/rosprov -i csv/example.csv -o target/provenance/example.provn
	target/bin/rosprov -i csv/example.csv -o target/provenance/example.pdf

clean:
	rm -rf target/libs target/bin target/provenance/*
