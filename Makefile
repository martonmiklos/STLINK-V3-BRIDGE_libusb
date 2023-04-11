.PHONY: all
all: build
	meson compile -C $<

build:
	meson setup $@

.PHONY: install
install: build
	meson install -C $<

.PHONY: install
install-local: build
	DESTDIR=$(CURDIR)/inst_root meson install -C $<

.PHONY: clean
clean:
	$(RM) -r build inst_root
