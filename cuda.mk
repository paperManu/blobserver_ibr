.cu.o:
	$(NVCC) -o $@ -c $<

.cu.lo:
	$(top_srcdir)/cudalt.py $@ $(NVCC) \
		--compiler-options=\"$(CFLAGS) \
		$(DEFAULT_INCLUDES) $(INCLUDES) \
		$(AM_CPPFLAGS) $(CPPFLAGS) \" -c $<
